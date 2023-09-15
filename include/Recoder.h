#ifndef CASRECODER_H
#define CASRECODER_H

#include <Utility.h>

using namespace etrs::utility;

namespace etrs::replay {
    class Recoder {
    private:
        string file_path;

    public:
        explict Recoder();
        ~Recoder();

        explict Recoder(const string file_path);
    };

} // namespace etrs::replay

#endif // CASNETWORK_H
