#include "jrdf/json.hpp"

using namespace json;

// Recursive helper function
std::shared_ptr<JsonValue> JsonParser::parseValue(const std::string& json, size_t& i) {
    while (std::isspace(json[i])) ++i;

    if (json[i] == '{') {
        ++i;
        JsonObject object;
        while (json[i] != '}') {
            std::string key;
            while (json[i] != ':') {
                if (json[i] != '"') {
                    key += json[i];
                }
                ++i;
            }
            ++i;
            object[key] = parseValue(json, i);
            if (json[i] == ',') ++i;
        }
        ++i;
        return std::make_shared<JsonValue>(object);
    } else if (json[i] == '[') {
        ++i;
        JsonArray array;
        while (json[i] != ']') {
            array.push_back(parseValue(json, i));
            if (json[i] == ',') ++i;
        }
        ++i;
        return std::make_shared<JsonValue>(array);
    } else if (json[i] == '"') {
        std::string value;
        ++i;
        while (json[i] != '"') {
            value += json[i++];
        }
        ++i;
        return std::make_shared<JsonValue>(value);
    } else {
        std::string value;
        while (json[i] != ',' && json[i] != '}' && json[i] != ']') {
            value += json[i++];
        }
        if (value == "null") {
            return std::make_shared<JsonValue>();
        } else if (value == "true") {
            return std::make_shared<JsonValue>(true);
        } else if (value == "false") {
            return std::make_shared<JsonValue>(false);
        } else {
            return std::make_shared<JsonValue>(std::stof(value));
        }
    }

    return nullptr; // Should never reach here
}

std::shared_ptr<JsonValue> JsonParser::parse(std::string json) {
    size_t i = 0;
    json.erase(remove_if(json.begin(), json.end(), isspace), json.end());
    return parseValue(json, i);
}

std::shared_ptr<JsonValue> JsonParser::parse(std::istream& jsonFile) {
    std::string json((std::istreambuf_iterator<char>(jsonFile)),
                     std::istreambuf_iterator<char>());
    return parse(json);
}

std::string JsonParser::stringify(const std::shared_ptr<JsonValue>& value) {
    switch (value->type()) {
        case JsonValue::Type::Null:
            return "null";
        case JsonValue::Type::String:
            return "\"" + value->as_string() + "\"";
        case JsonValue::Type::Float:
            return std::to_string(value->as_float());
        case JsonValue::Type::Boolean:
            return value->as_boolean() ? "true" : "false";
        case JsonValue::Type::Array: {
            std::string result = "[";
            const JsonArray& array = value->as_array();
            for (size_t i = 0; i < array.size(); ++i) {
                if (i != 0) {
                    result += ",";
                }
                result += stringify(array[i]);
            }
            result += "]";
            return result;
        }
        case JsonValue::Type::Object: {
            std::string result = "{";
            const JsonObject& object = value->as_object();
            bool first = true;
            for (const auto& pair : object) {
                if (!first) {
                    result += ",";
                }
                first = false;
                result += "\"" + pair.first + "\":" + stringify(pair.second);
            }
            result += "}";
            return result;
        }
    }
    return ""; // Should never reach here
}