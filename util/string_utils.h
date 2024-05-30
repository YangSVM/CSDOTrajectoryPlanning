//
// Created by yenkn on 2021/2/2.
//
#pragma once
#include <sstream>

namespace common {

template<typename T>
void StrCat(std::stringstream &ss, T value)
{
  ss << value;
}

template<typename T, typename... Args>
void StrCat(std::stringstream &ss, T value, Args... args)
{
  ss << value;
  StrCat(ss, args...);
}

template<typename... Args>
std::string StrCat(Args... args)
{
  std::stringstream ss;
  StrCat(ss, args...);
  return ss.str();
}

}