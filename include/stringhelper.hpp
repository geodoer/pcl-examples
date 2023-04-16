#pragma once
#include<string>

#include<boost/locale.hpp>

class StringHelper
{
public:
	// 依赖Boost::locate模块
	static std::string utf8ToGbk(const std::string& utf8_str)
	{
		return boost::locale::conv::from_utf(utf8_str.c_str(), std::string("gb2312"));
	}
	static std::string gbkToUtf8(const std::string& gbk_str)
	{
		return boost::locale::conv::to_utf<char>(gbk_str.c_str(), std::string("gb2312"));
	}
};