#include "Config.h"

#include <fstream>
#include <cstring>
#include <string>
#include <boost/algorithm/string.hpp>

bool Config::parseSimple(const std::string& path) {
    std::ifstream conf(path);

    if (!conf.is_open()) {
        return false;
    }

    std::string line;
    while (std::getline(conf, line)) {
		std::string lineCopy(line);
		boost::trim(lineCopy);
		if (lineCopy.length() == 0) {
			continue;
		}
        auto pos = line.find(':');
        auto key = line.substr(0, pos);
        auto value = line.substr(pos + 2);

        if (value.length() ==  0) {
            continue;
        }

		if (key[0] == '#') {
			continue;
		}

        data[key] = value;

        std::cout << "Config: " << key << " -> " << value << "\n";
    }

    conf.close();

    return true;
}


std::string Config::getValue(const std::string& key) {
    if (data.find(key) != data.end()) {
		auto value = data[key];
		value = std::string(value.substr(1, value.length() - 2));
		return value;
    }
    return "";
}

int Config::getValueI(const std::string& key) {
	if (data.find(key) != data.end()) {
		return std::stoi(data[key]);
	}
	return 0;
}

float Config::getValueF(const std::string& key) {
	if (data.find(key) != data.end()) {
		return std::stof(data[key]);
	}
	return INFINITY;
}
