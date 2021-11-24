#ifndef TEST_FEC_HPP
#define TEST_FEC_HPP
#ifdef TEST_FEC

static constexpr int INPUT_ALTERED_ERR = 20;
static constexpr int VECTOR_WRONG_SIZE_ERR = 21;
static constexpr int VECTOR_ALTERED = 22;
static constexpr int DATA_MISMATCH = 23;

using namespace std;

template<typename T> 
bool test_fec(const size_t pld_len, const int conv_order, const int conv_rate, const int num_rs_bytes,
                    const double p_flip) {
    // Generate random data
    mt19937 rand_data(time(0));
    uniform_int_distribution<uint8_t> dist(0, 255);
    vector<uint8_t> orig_test_data(pld_len, 0);
    for(auto & td : orig_test_data) {
        td = dist(rand_data);
    }
    // Encode it with one of the FEC objects
    mt19937 rand_size(time(0));
    uniform_int_distribution<size_t> arr_sizes(0, 256);
    auto fec_enc = make_shared<T>(pld_len, conv_rate, conv_order, num_rs_bytes);
    vector<uint8_t> orig_enc_test_data(arr_sizes(rand_size), 0);
    vector<uint8_t> test_data = orig_test_data;
    fec_enc->encode(test_data, orig_enc_test_data);
    if(test_data != orig_test_data) {
        cout << "ERROR: FEC has inappropriately altered the input data!" << endl;
        throw INPUT_ALTERED_ERR;
    }
    if(fec_enc->encSize() != orig_enc_test_data.size()) {
        cout << "ERROR: Encoded test data vector is the wrong size!" << endl;
        throw VECTOR_WRONG_SIZE_ERR;
    }
    // Randomly flip bits
#if 1
    mt19937 rand_flip(time(0));
    uniform_real_distribution<double> p_dist(0.0, 1.0);
    for(auto & td : orig_enc_test_data) {
        static constexpr int BITS_IN_BYTE = 8;
        for(int i = 0; i < BITS_IN_BYTE; i++) {
            if(p_dist(rand_flip) < p_flip) {
                td ^= 0x01 << i;
            }
        }
    }
#endif
    // Attempt to decode the altered bits with a "separate" FEC
    auto fec_dec = make_shared<T>(pld_len, conv_rate, conv_order, num_rs_bytes);
    if(fec_dec->encSize() != orig_enc_test_data.size()) {
        cout << "ERROR: size of FEC-encoded byte vector not the same as the FEC's stated encoded size" << endl;
        throw VECTOR_WRONG_SIZE_ERR;
    }
    vector<uint8_t> dec_test_data(arr_sizes(rand_size), 0);
    vector<uint8_t> enc_test_data = orig_enc_test_data;
    fec_dec->decode(enc_test_data, dec_test_data);
    if(enc_test_data != orig_enc_test_data) {
        cout << "ERROR: enc_test_data was altered by the decode method " << __LINE__ << endl;
        throw VECTOR_ALTERED;
    }
    if(pld_len != dec_test_data.size()) {
        cout << "ERROR: size of decoded vector is not the same as the payload length " << __LINE__ << endl;
        throw VECTOR_WRONG_SIZE_ERR;
    }
    if(orig_test_data.size() != dec_test_data.size()) {
        cout << "ERROR: size of original and decoded vector not the same " << __LINE__ << endl;
        throw VECTOR_WRONG_SIZE_ERR;
    }
    // Attempt to decode the altered bits with the "original" FEC
    if(fec_enc->encSize() != orig_enc_test_data.size()) {
        cout << "ERROR: size of FEC-encoded byte vector not the same as the FEC's stated encoded size " << __LINE__ << endl;
        throw VECTOR_WRONG_SIZE_ERR;
    }
    vector<uint8_t> dec_test_data2(arr_sizes(rand_size), 0);
    vector<uint8_t> enc_test_data2 = orig_enc_test_data;
    fec_dec->decode(enc_test_data2, dec_test_data2);
    if(enc_test_data2 != orig_enc_test_data) {
        cout << "ERROR: enc_test_data was altered by the decode method " << __LINE__ << endl;
        throw VECTOR_ALTERED;
    }
    if(pld_len != dec_test_data2.size()) {
        cout << "ERROR: size of decoded vector is not the same as the payload length " << __LINE__ << endl;
        throw VECTOR_WRONG_SIZE_ERR;
    }
    if(orig_test_data.size() != dec_test_data2.size()) {
        cout << "ERROR: size of original and decoded vector not the same " << __LINE__ << endl;
        throw VECTOR_WRONG_SIZE_ERR;
    }
    // Compare the final results
    if(dec_test_data != dec_test_data2) {
        cout << "ERROR: decoded data does not match " << __LINE__ << endl;
        throw DATA_MISMATCH;
    }
#if 0
    for(int i = 0; i < orig_test_data.size(); i++) {
        if(orig_test_data != dec_test_data) {
            for(int i = 0; i < orig_test_data.size(); i++) {
                cout << int(orig_test_data[i]) << ":" << int(dec_test_data[i]) << " ";
            }
            cout << endl;
        }
    }
#endif
    return orig_test_data == dec_test_data;
}


template<typename T>
void test_fec() {
    static vector<int> conv_orders = { 6, 7, 9 };
    static vector<int> conv_rates = { 2, 3 };
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
                            if(test_fec<T>(pld_len, conv_order, conv_rate, rs_byte, p_flip)) {
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

#endif /* TEST_FEC */

#endif /* TEST_FEC_HPP */