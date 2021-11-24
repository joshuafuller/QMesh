#ifdef TEST_FEC

#include "fec.hpp"
#include <vector>
#include <chrono>
#include <iostream>
#include <memory>
#include "test_fec.hpp"

using namespace std;

void test_fec();
void test_fec() {
    static vector<int> conv_orders = { 7 };
    static vector<int> conv_rates = { 2 };
    static vector<int> rs_bytes = { 2, 3, 4, 5, 6, 7, 8 };
    static vector<double> p_flips = { 0.00, 0.01, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30 };
    static pair<int, int> pld_len_range(16, 64);
    static int num_trials = 100;

    for(int pld_len = pld_len_range.first; pld_len < pld_len_range.second; pld_len++) {
        for(auto & conv_order : conv_orders) {
            for(auto & conv_rate : conv_rates) {
                for(auto & rs_byte : rs_bytes) {
                    for(auto & p_flip : p_flips) {
                        int num_correct = 0;
                        int num_total = 0;
                        for(int i = 0; i < num_trials; i++) {
                            if(test_fec<FECInterleave>(pld_len, conv_order, conv_rate, rs_byte, p_flip)) {
                                num_correct += 1;
                            }
                            num_total += 1;
                        }
                        cout << "Pld Len: " << pld_len << " Order: " << conv_order << " Rate: "
                            << conv_rate << " RS Bytes: " << rs_byte << " Pflip: " << p_flip << 
                            "; Num corr " << num_correct << " out of " << num_total << endl; 
                    }
                }
            }
        }
    }
}

int main() {
    test_fec();

    return 0;
}


#endif /* TEST_FEC */