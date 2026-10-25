#include "rollchain_model/naming.hpp"

namespace rollchain_model
{

std::vector<std::string> urdf_joint_names(std::size_t num_joints)
{
  std::vector<std::string> names;
  names.reserve(num_joints);
  for (std::size_t i = 1; i <= num_joints; ++i)
  {
    names.push_back("j" + std::to_string(i) + "_roll");
  }
  return names;
}

}  // namespace rollchain_model
