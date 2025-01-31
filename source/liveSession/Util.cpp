#include "Util.hpp"

#include <gz/common/Util.hh>

namespace gz::omniverse
{
std::string validPath(const std::string &_path)
{
  std::string result;
  if (_path.empty())
  {
    return result;
  }
  result = gz::common::replaceAll(_path, " ", "");
  result = gz::common::replaceAll(result, ".", "_");
  result = gz::common::replaceAll(result, "-", "_");
  result = gz::common::replaceAll(result, "<", "_");
  result = gz::common::replaceAll(result, ">", "_");
  result = gz::common::replaceAll(result, "[", "_");
  result = gz::common::replaceAll(result, "]", "_");
  if (std::isdigit(result[0]))
  {
    result = "_" + result;
  }
  return result;
}
}  // namespace gz::omniverse
