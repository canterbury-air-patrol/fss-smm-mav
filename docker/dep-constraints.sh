#!/bin/bash -e

# Report the Canterbury Air Patrol dependencies the image needs: as Debian
# dependency terms ready for `apt-get satisfy`, as bare package names, or as the
# versions currently installed.
#
#   dep-constraints.sh                  build-stage terms (the -dev packages)
#   dep-constraints.sh --runtime        runtime-stage terms (the shared libs)
#   dep-constraints.sh --packages       bare package names, one per line
#   dep-constraints.sh --versions       "module<TAB>version" for what is installed
#
# The bounds are *derived from configure.ac*, not written here. That is the
# whole point of the script: the fss-client-ssl floor is a semantic requirement
# (see configure.ac's comment on the 1.3.0 break), and until this existed it was
# enforced only where the source was compiled. `apt install -y
# libfss-client-ssl` reaches apt.canterburyairpatrol.org, which is a rolling
# repository publishing exactly one version of each package, so the image build
# took whatever was newest and configure then checked a floor the install had
# already sailed past. A future release outside the window would have been
# installed silently and only the *floor* would have complained.
#
# Deriving means a window added to configure.ac reaches the image build in the
# same commit -- there is no second copy of the bounds to forget. When the next
# ABI break is announced and configure.ac's `>=` becomes a window again, this
# script starts emitting the ceiling with no edit.
#
# Only top-level PKG_CHECK_MODULES calls are read. Catch2's are indented inside
# an AS_IF and are deliberately skipped: catch2 comes from Debian, not from the
# rolling repository, so it is not what this guards.

CONFIGURE_AC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/configure.ac"

mode=constraints
runtime=0
for arg in "$@"; do
	case "$arg" in
	--runtime) runtime=1 ;;
	--packages) mode=packages ;;
	--versions) mode=versions ;;
	*)
		echo "dep-constraints.sh: unknown argument '${arg}'" >&2
		exit 1
		;;
	esac
done

# pkg-config module name -> Debian binary package. Unavoidable duplication: the
# two namespaces are genuinely different, and debian/control carries the same
# mapping. Modules with no entry (jsoncpp) are Debian's own; the build stage
# installs it unconstrained (docker/setup.sh) and the runtime stage gets it as a
# dependency of libfss-client-ssl, which uses it in its own API.
#
# smm-asset is the one module whose two stages want different packages:
# libsmm-asset-dev is headers plus the -dev dependency chain (libtidy-dev,
# libjansson-dev, libcurl4-openssl-dev), none of which belongs in a flight
# image, and libsmmasset0 is the shared library the binary actually loads. The
# two are published from one source at one version -- libsmm-asset-dev depends
# on `libsmmasset0 (= <same version>)' -- which is what makes the cross-stage
# version comparison in docker/runtime-setup.sh meaningful. The FSS packages
# need no such split: libfss-client-ssl and libfss-transport are themselves the
# runtime packages (debian/control's `Depends' names them), and are what
# `cap-fmu' the .deb depends on.
deb_package() {
	case "$1" in
	fss-client-ssl) echo "libfss-client-ssl" ;;
	fss-transport) echo "libfss-transport" ;;
	smm-asset)
		if [ "$runtime" -eq 1 ]; then
			echo "libsmmasset0"
		else
			echo "libsmm-asset-dev"
		fi
		;;
	*) echo "" ;;
	esac
}

# pkg-config comparison operator -> dpkg's. dpkg spells the strict comparisons
# `<<' and `>>'; `<' and `>' are accepted but mean <= and >=, which would
# silently widen a ceiling by one release.
dpkg_operator() {
	case "$1" in
	'>=') echo ">=" ;;
	'>') echo ">>" ;;
	'<=') echo "<=" ;;
	'<') echo "<<" ;;
	'=') echo "=" ;;
	*) echo "" ;;
	esac
}

emitted=""
while read -r spec; do
	read -r -a tokens <<<"$spec"
	i=0
	while [ "$i" -lt "${#tokens[@]}" ]; do
		module="${tokens[$i]}"
		i=$((i + 1))
		operator=""
		version=""
		case "${tokens[$i]:-}" in
		'>=' | '>' | '<=' | '<' | '=')
			operator="${tokens[$i]}"
			version="${tokens[$((i + 1))]}"
			i=$((i + 2))
			;;
		esac

		package="$(deb_package "$module")"
		[ -n "$package" ] || continue
		emitted="${emitted} ${module}"

		case "$mode" in
		packages)
			echo "$package"
			continue
			;;
		versions)
			# Deliberately keyed by pkg-config module, not by package:
			# the whole reason this mode exists is to compare the build
			# stage against the runtime stage, and those two stages
			# install different package names for the same module.
			if ! installed="$(dpkg-query -W -f='${Version}' "$package" 2>/dev/null)"; then
				echo "dep-constraints.sh: ${package} is not installed" >&2
				exit 1
			fi
			printf '%s\t%s\n' "$module" "$installed"
			continue
			;;
		esac

		if [ -z "$operator" ]; then
			echo "$package"
			continue
		fi

		dpkg_op="$(dpkg_operator "$operator")"
		if [ -z "$dpkg_op" ]; then
			echo "dep-constraints.sh: unhandled comparison '${operator}' on ${module} in configure.ac" >&2
			exit 1
		fi
		echo "${package} (${dpkg_op} ${version})"
	done
done < <(sed -n 's/^PKG_CHECK_MODULES(\[[^]]*\], *\[\([^]]*\)\].*/\1/p' "$CONFIGURE_AC")

# A parse that silently matched nothing would hand `apt-get satisfy` an empty
# argument list, which succeeds having constrained nothing -- reinstating
# exactly the unpinned install this script exists to remove. Fail loudly
# instead: if configure.ac's shape changes, this stops the build rather than
# quietly stopping guarding.
for required in fss-client-ssl fss-transport smm-asset; do
	case " ${emitted} " in
	*" ${required} "*) ;;
	*)
		echo "dep-constraints.sh: no bound found for ${required} in ${CONFIGURE_AC}" >&2
		exit 1
		;;
	esac
done
