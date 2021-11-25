#ifdef TEST_FEC

#include "fec.hpp"
#include <vector>
#include <chrono>
#include <iostream>
#include <memory>
#include "test_fec.hpp"

using namespace std;

int main() {
    cout << "================================================================================" << endl;
    cout << "NOW TESTING FECRSV" << endl;
    test_fec<FECRSV>();
    cout << "================================================================================" << endl;
    cout << "NOW TESTING FECConv" << endl;
    test_fec<FECConv>();
    cout << "================================================================================" << endl;
    cout << "NOW TESTING FECInterleave" << endl;
    test_fec<FECInterleave>();
#if 0
    cout << "================================================================================" << endl;
    cout << "NOW TESTING FECRSVGolay" << endl;
    test_fec<FECRSVGolay>();
#endif
    return 0;
}


#endif /* TEST_FEC */