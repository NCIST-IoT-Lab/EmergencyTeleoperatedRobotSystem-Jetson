#ifndef CASRECODER_H
#define CASRECODER_H

#include <Utility.h>

using namespace etrs::utility;

namespace etrs {
    namespace replay {
        class Recoder {
        private:
            string file_path;
        public:
            Recoder();
            ~Recoder();

            Recoder(const string file_path);
        };

    }; // namespace replay
} // namespace etrs

#endif // CASNETWORK_H
