#ifndef _RECODER_H_
#define _RECODER_H_

#include <Utility.h>

using namespace etrs::utility;

namespace etrs::replay {
    class Recoder {
    private:
        string file_path;

    public:
        explicit Recoder();
        ~Recoder();

        explicit Recoder(const string file_path);
    };

} // namespace etrs::replay

#endif // _RECODER_H_
