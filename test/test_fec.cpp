#ifdef TEST_FEC

#include "fec.hpp"
#include <vector>
#include <chrono>
#include <iostream>
#include <memory>
#include "test_fec.hpp"

using namespace std;

int main() {
    test_fec<FECRSV>();
    test_fec<FECConv>();
    test_fec<FECInterleave>();
    test_fec<FECRSVGolay>();

    return 0;
}


#endif /* TEST_FEC */