#ifndef MISC_STRINGOPS_H
#define MISC_STRINGOPS_H

#include <cctype>
#include <string>
#include <algorithm>

namespace Misc
{
class StringUtils
{
    struct ci
    {
        bool operator()(int x, int y) const {
            return std::tolower(x) < std::tolower(y);
        }
    };

public:
    static bool ciLess(const std::string &x, const std::string &y) {
        return std::lexicographical_compare(x.begin(), x.end(), y.begin(), y.end(), ci());
    }

    static bool ciEqual(const std::string &x, const std::string &y) {
        if (x.size() != y.size()) {
            return false;
        }
        std::string::const_iterator xit = x.begin();
        std::string::const_iterator yit = y.begin();
        for (; xit != x.end(); ++xit, ++yit) {
            if (std::tolower(*xit) != std::tolower(*yit)) {
                return false;
            }
        }
        return true;
    }

    static int ciCompareLen(const std::string &x, const std::string &y, size_t len)
    {
        std::string::const_iterator xit = x.begin();
        std::string::const_iterator yit = y.begin();
        for(;xit != x.end() && yit != y.end() && len > 0;++xit,++yit,--len)
        {
            int res = *xit - *yit;
            if(res != 0 && std::tolower(*xit) != std::tolower(*yit))
                return (res > 0) ? 1 : -1;
        }
        if(len > 0)
        {
            if(xit != x.end())
                return 1;
            if(yit != y.end())
                return -1;
        }
        return 0;
    }

    /// Transforms input string to lower case w/o copy
    static std::string &toLower(std::string &inout) {
        std::transform(
            inout.begin(),
            inout.end(),
            inout.begin(),
            ::tolower
        );
        return inout;
    }

    /// Returns lower case copy of input string
    static std::string lowerCase(std::string str)
    {
        std::transform(
            str.begin(),
            str.end(),
            str.begin(),
            ::tolower
        );
        return str;
    }
};


/// Returns true if str1 begins with substring str2
bool begins(const char* str1, const char* str2);

/// Returns true if str1 ends with substring str2
bool ends(const char* str1, const char* str2);

/// Case insensitive, returns true if str1 begins with substring str2
bool ibegins(const char* str1, const char* str2);

/// Case insensitive, returns true if str1 ends with substring str2
bool iends(const char* str1, const char* str2);

}

#endif
