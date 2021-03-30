#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include <map>

class Config {

private:
  Config() = default; // Disallow instantiation outside of the class.

  std::map<std::string, std::string> data;

public:
  Config(const Config&) = default;
  Config& operator=(const Config &) = delete;
  Config(Config &&) = delete;
  Config& operator=(Config &&) = delete;

  static auto& instance() {
      static Config inst;
      return inst;
  }

  bool parseSimple(const std::string& path);

  std::string getValue(const std::string& key);
  int getValueI(const std::string& key);
  float getValueF(const std::string& key);
};

#endif // CONFIG_H
