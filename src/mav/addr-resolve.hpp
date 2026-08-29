#pragma once
#include <arpa/inet.h>
#include <cstdint>
#include <cstring>
#include <netdb.h>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <vector>

/* Turning a resolved MAV endpoint into the list of addresses to dial.
 *
 * The MAV endpoint is resolved with AF_UNSPEC, which is precisely the case
 * where a host name answers with several addresses -- typically an AAAA ahead
 * of an A, since glibc's RFC 6724 sorting prefers IPv6 when the host has a
 * global v6 address configured. Using only the first of them means an address
 * that is resolvable but not routable to the autopilot (a stale AAAA, a
 * dual-homed MAVProxy bound on v4 only) is a permanent connect failure that the
 * reconnector retries against the same dead address forever, without ever
 * trying the one that works.
 *
 * The list-building and formatting below are separated from getaddrinfo() so
 * they can be unit tested over a hand-built addrinfo chain, with no resolver
 * and no network -- the same reason mav-comms.hpp's predicates are their own
 * header. connect_to_mav() supplies the iteration. */

/* Length of the sockaddr for `family`, which is what connect() needs and what
 * sockaddr_storage (deliberately oversized) does not carry. Only the two
 * families sockaddr_endpoints() emits are answered; anything else gives 0. */
inline auto
sockaddr_len_for_family (int family) -> socklen_t
{
    switch (family)
    {
        case AF_INET:
            return sizeof (struct sockaddr_in);
        case AF_INET6:
            return sizeof (struct sockaddr_in6);
        default:
            return 0;
    }
}

/* Overwrite the port in an already-populated sockaddr. getaddrinfo() is called
 * with a null service (the port is configuration, not part of the resolved
 * name), so every result comes back with port 0 and needs this. Returns false
 * for any family other than AF_INET/AF_INET6, which is what keeps such a result
 * out of the dial list rather than dialling a sockaddr whose port was never
 * set. */
inline auto
set_sockaddr_port (struct sockaddr_storage *sa, uint16_t port) -> bool
{
    switch (sa->ss_family)
    {
        case AF_INET:
        {
            auto *sa_in = reinterpret_cast<struct sockaddr_in *> (sa);
            sa_in->sin_port = htons (port);
            return true;
        }
        case AF_INET6:
        {
            auto *sa_in6 = reinterpret_cast<struct sockaddr_in6 *> (sa);
            sa_in6->sin6_port = htons (port);
            return true;
        }
        default:
            return false;
    }
}

/* Every dialable address in a getaddrinfo() result chain, in the order the
 * resolver returned them (which is already sorted by RFC 6724 destination
 * preference -- a better ordering than anything this end could impose), each
 * with `port` written into it.
 *
 * Results that cannot be dialled are dropped rather than being allowed to fail
 * a connect: a family that is neither AF_INET nor AF_INET6, or an ai_addrlen
 * that does not fit a sockaddr_storage. A null ai_addr is treated the same way.
 * An empty result therefore means "nothing here is usable", which the caller
 * reports exactly as it reports a resolution failure.
 *
 * The vector's element type is spelled without the `struct` keyword -- elsewhere
 * in this tree these types carry it -- because cppcheck (the check-code.sh gate)
 * misparses an elaborated type specifier inside a template argument list and
 * reports the statement after it as a syntax error. The two spellings are the
 * same type to the compiler. */
inline auto
sockaddr_endpoints (const struct addrinfo *ai, uint16_t port) -> std::vector<sockaddr_storage>
{
    std::vector<sockaddr_storage> endpoints;
    for (const struct addrinfo *cur = ai; cur != nullptr; cur = cur->ai_next)
    {
        if (cur->ai_addr == nullptr || cur->ai_addrlen == 0 || cur->ai_addrlen > sizeof (struct sockaddr_storage))
        {
            continue;
        }
        struct sockaddr_storage sa = {};
        memcpy (&sa, cur->ai_addr, cur->ai_addrlen);
        /* Trust ai_addr's own family over ai_family: they agree for every
         * result a resolver produces, and it is the bytes that were copied. */
        if (sockaddr_len_for_family (sa.ss_family) == 0 || !set_sockaddr_port (&sa, port))
        {
            continue;
        }
        endpoints.push_back (sa);
    }
    return endpoints;
}

/* A resolved address in a form fit for a log line: `1.2.3.4:5760`, or
 * `[2001:db8::1]:5760` for v6 (the brackets are what keep the port readable
 * next to a v6 address's own colons). Used to say which of several addresses
 * an attempt was against, so a flight log shows the one that failed as well as
 * the one that worked. Never throws and never returns empty: an address it
 * cannot format at all comes back as `<unknown>`, since this only ever feeds a
 * message. */
inline auto
describe_sockaddr (const struct sockaddr_storage &sa) -> std::string
{
    char host[INET6_ADDRSTRLEN] = { 0 };
    switch (sa.ss_family)
    {
        case AF_INET:
        {
            const auto *sa_in = reinterpret_cast<const struct sockaddr_in *> (&sa);
            if (inet_ntop (AF_INET, &sa_in->sin_addr, host, sizeof (host)) == nullptr)
            {
                return "<unknown>";
            }
            return std::string (host) + ":" + std::to_string (ntohs (sa_in->sin_port));
        }
        case AF_INET6:
        {
            const auto *sa_in6 = reinterpret_cast<const struct sockaddr_in6 *> (&sa);
            if (inet_ntop (AF_INET6, &sa_in6->sin6_addr, host, sizeof (host)) == nullptr)
            {
                return "<unknown>";
            }
            return "[" + std::string (host) + "]:" + std::to_string (ntohs (sa_in6->sin6_port));
        }
        default:
            return "<unknown>";
    }
}
