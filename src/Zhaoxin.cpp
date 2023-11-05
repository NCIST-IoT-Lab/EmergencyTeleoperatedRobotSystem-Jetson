#include "DataMessage.pb.h"
#include "Network.h"

#include <iostream>
#include <string>

#include <thread>
#include <vector>

#include <open3d/Open3D.h>

using namespace std;
using namespace open3d;
using namespace etrs::utility;

int main(int argc, char **argv) {

    etrs::net::BaseCommunicator lpcom(924);
    lpcom.createServerSocket();
    cout << "server socket created" << endl;
    lpcom.acceptConnection();

    geometry::TriangleMesh ply;
    io::ReadTriangleMesh("ply/sm.ply", ply);
    
    // lpcom.sendData((const unsigned char*)ply_str.c_str(), ply_str.length());
}
