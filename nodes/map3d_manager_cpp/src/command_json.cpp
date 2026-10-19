#include "map3d_manager_cpp/command_json.hpp"

#include <algorithm>
#include <cstddef>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>
#include <variant>

namespace map3d_manager_cpp
{

namespace
{

using nlohmann::json;

bool valid_name(const std::string & name)
{
  return !name.empty() && name.size() <= MAX_NAME_LEN && std::all_of(name.begin(), name.end(), [](unsigned char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
  });
}

}  // namespace

const char * action_name(Action action)
{
  switch (action) {
    case Action::SAVE:
      return "save";
    case Action::RESET:
      return "reset";
    case Action::PAUSE:
      return "pause";
    case Action::RESUME:
      return "resume";
  }
  return "";
}

const char * status_name(Status status)
{
  switch (status) {
    case Status::OK:
      return "ok";
    case Status::ERROR:
      return "error";
    case Status::BUSY:
      return "busy";
    case Status::INVALID:
      return "invalid";
    case Status::TIMEOUT:
      return "timeout";
  }
  return "";
}

std::variant<Command, ParseError> parse_command(const std::string & text)
{
  const json doc = json::parse(text, nullptr, false);
  if (doc.is_discarded() || !doc.is_object()) {
    return ParseError{std::nullopt, "not a JSON object"};
  }
  const auto id_it = doc.find("request_id");
  if (id_it == doc.end() || !id_it->is_string()) {
    return ParseError{std::nullopt, "request_id missing or not a string"};
  }
  const std::string request_id = id_it->get<std::string>();
  if (request_id.empty() || request_id.size() > MAX_REQUEST_ID_LEN) {
    return ParseError{std::nullopt, "request_id must be 1..64 characters"};
  }

  Command command;
  command.request_id = request_id;
  const auto action_it = doc.find("action");
  if (action_it == doc.end() || !action_it->is_string()) {
    return ParseError{request_id, "action missing or not a string"};
  }
  const std::string action = action_it->get<std::string>();
  bool known = false;
  for (const Action candidate : {Action::SAVE, Action::RESET, Action::PAUSE, Action::RESUME}) {
    if (action == action_name(candidate)) {
      command.action = candidate;
      known = true;
    }
  }
  if (!known) {
    return ParseError{request_id, "unknown action '" + action + "' (save, reset, pause, resume)"};
  }
  if (const auto name_it = doc.find("name"); name_it != doc.end()) {
    if (!name_it->is_string() || !valid_name(name_it->get<std::string>())) {
      return ParseError{request_id, "name must match [A-Za-z0-9_-]{1,64}"};
    }
    command.name = name_it->get<std::string>();
  }
  return command;
}

std::string to_json(const Result & result)
{
  json doc = {
    {"request_id", result.request_id},
    {"status", status_name(result.status)},
    {"message", result.message},
    {"action", result.action}};
  if (result.db_path) {
    doc["db_path"] = *result.db_path;
  }
  if (result.db_size_mb) {
    doc["db_size_mb"] = *result.db_size_mb;
  }
  return doc.dump();
}

}  // namespace map3d_manager_cpp
