#pragma once
#include <string>

class SMMSettings {
private:
    std::string url;
    std::string user;
    std::string pass;
public:
    SMMSettings(std::string t_url, std::string t_user, std::string t_pass) : url(t_url), user(t_user), pass(t_pass) {};
    std::string getURL() { return this->url; };
    std::string getUsername() { return this->user; };
    std::string getPassword() { return this->pass; };
};
