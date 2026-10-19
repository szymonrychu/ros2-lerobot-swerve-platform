// The JSON contract of the 3D map commands, shared by the manager node, the web UI and the agent.
//
// Command, std_msgs/String on command_topic (/uplink/map3d/command):
//   {"request_id": str (1..64 chars), "action": "save"|"reset"|"pause"|"resume",
//    "name": str (optional, [A-Za-z0-9_-]{1,64}, default "slam_map")}
//
// Result, std_msgs/String on result_topic (/map3d/result_server), one per accepted or rejected command:
//   {"request_id": str, "status": "ok"|"error"|"busy"|"invalid"|"timeout", "message": str, "action": str,
//    "db_path": str (optional), "db_size_mb": number (optional)}
//
// A command that is not JSON, not an object, or has no usable request_id (missing, not a string, empty or longer than
// 64 chars) is dropped without a result. Any other fault (unknown action, bad name, wrong type) is answered with an
// "invalid" result that carries the request_id, so the sender can match it.
#ifndef MAP3D_MANAGER_CPP__COMMAND_JSON_HPP_
#define MAP3D_MANAGER_CPP__COMMAND_JSON_HPP_

#include <optional>
#include <string>
#include <variant>

namespace map3d_manager_cpp
{

constexpr std::size_t MAX_REQUEST_ID_LEN = 64;
constexpr std::size_t MAX_NAME_LEN = 64;
constexpr const char * DEFAULT_MAP_NAME = "slam_map";

enum class Action
{
  SAVE,
  RESET,
  PAUSE,
  RESUME
};

enum class Status
{
  OK,
  ERROR,
  BUSY,
  INVALID,
  TIMEOUT
};

// Wire name of an action ("save", ...) / status ("ok", ...).
const char * action_name(Action action);
const char * status_name(Status status);

// A valid command.
struct Command
{
  std::string request_id;
  Action action = Action::SAVE;
  std::string name = DEFAULT_MAP_NAME;
};

// A rejected command: request_id is set when it was usable (answer with an "invalid" result), empty when the command
// is dropped. message says what is wrong.
struct ParseError
{
  std::optional<std::string> request_id;
  std::string message;
};

// A command's answer; db_path and db_size_mb are set only by a successful save.
struct Result
{
  std::string request_id;
  Status status = Status::OK;
  std::string message;
  std::string action;
  std::optional<std::string> db_path;
  std::optional<double> db_size_mb;
};

// Parse the text of a command message.
// @param text the std_msgs/String data
// @return the Command, or the ParseError
std::variant<Command, ParseError> parse_command(const std::string & text);

// Serialise a Result to its JSON text (optional fields omitted when unset).
std::string to_json(const Result & result);

}  // namespace map3d_manager_cpp

#endif  // MAP3D_MANAGER_CPP__COMMAND_JSON_HPP_
