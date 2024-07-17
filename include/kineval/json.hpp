#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <iterator>
#include <memory>

namespace kineval {

class JsonValue;

typedef std::map<std::string, std::shared_ptr<JsonValue>> JsonObject;
typedef std::vector<std::shared_ptr<JsonValue>> JsonArray;

class JsonValue {
public:
    enum class Type {
        Null,
        String,
        Float,
        Boolean,
        Array,
        Object
    };

    JsonValue() : type_(Type::Null) {}
    JsonValue(const std::string& value) : type_(Type::String), string_value_(value) {}
    JsonValue(float value) : type_(Type::Float), float_value_(value) {}
    JsonValue(bool value) : type_(Type::Boolean), boolean_value_(value) {}
    JsonValue(const JsonArray& value) : type_(Type::Array), array_value_(value) {}
    JsonValue(const JsonObject& value) : type_(Type::Object), object_value_(value) {}

    Type type() const { return type_; }

    std::string as_string() const { return string_value_; }
    double as_float() const { return float_value_; }
    bool as_boolean() const { return boolean_value_; }
    const JsonArray& as_array() const { return array_value_; }
    const JsonObject& as_object() const { return object_value_; }

private:
    Type type_;
    std::string string_value_;
    float float_value_;
    bool boolean_value_;
    JsonArray array_value_;
    JsonObject object_value_;
};

class JsonParser {
public:
    static std::shared_ptr<JsonValue> parse(std::istream& jsonFile);
    static std::shared_ptr<JsonValue> parse(std::string json);
    static std::string stringify(const std::shared_ptr<JsonValue>& value);
private:
    static std::shared_ptr<JsonValue> parseValue(const std::string& json, size_t& index);
};
} // namespace json