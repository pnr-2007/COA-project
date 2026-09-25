#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <typeinfo>
#include <cstdint>
#include <cmath>
#include <stdexcept>

using namespace std;

class ALU {

private:

    struct ALUFlags {
        bool zero = false;
        bool sign = false;
        bool overflow = false;
        bool carry = false;
        bool sizematch = true;
    };

    ALUFlags flags;

public:

    ALU() = default;
    ~ALU() = default;

    int add(int a, int b) {
        return a + b;
    }

    int sub(int a, int b) {
        return a - b;
    }

    int mul(int a, int b) {
        return a * b;
    }

    int div(int a, int b) {
        if (b != 0) {
            return a / b;
        }

        flags.zero = true;
        cerr << "Division by zero error" << endl;
        return -1;
    }

    int mod(int a, int b) {
        if (b != 0) {
            return a % b;
        }

        flags.zero = true;
        cerr << "Division by zero error" << endl;
        return -1;
    }

    int and_op(int a, int b) {
        return a & b;
    }

    int or_op(int a, int b) {
        return a | b;
    }

    int xor_op(int a, int b) {
        return a ^ b;
    }

    int not_op(int a) {
        return ~a;
    }

    uint32_t lsl(int a, int b) {
        return static_cast<uint32_t>(a) << b;
    }

    uint32_t lsr(int a, int b) {
        return static_cast<uint32_t>(a) >> b;
    }

    int asr(int a, int b) {
        return a >> b;
    }

    uint32_t asl(int a, int b) {
        return static_cast<uint32_t>(a) << b;
    }

    float sin(float a) {
        return std::sin(a);
    }

    float cos(float a) {
        return std::cos(a);
    }

    float log(float a) {
        return std::log(a);
    }

    float exp(float a) {
        return std::exp(a);
    }

    float pow(float a, float b) {
        return std::pow(a, b);
    }

    vector<vector<int>> matrix_add(
        const vector<vector<int>>& A,
        const vector<vector<int>>& B
    ) {
        if (A.empty() || B.empty() ||
            A.size() != B.size() ||
            A[0].size() != B[0].size()) {

            flags.sizematch = false;
            cerr << "Matrix size mismatch" << endl;
            return {};
        }

        flags.sizematch = true;

        int rows = A.size();
        int cols = A[0].size();

        vector<vector<int>> C(
            rows,
            vector<int>(cols)
        );

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                C[i][j] = A[i][j] + B[i][j];
            }
        }

        return C;
    }

    vector<vector<int>> matrix_sub(
        const vector<vector<int>>& A,
        const vector<vector<int>>& B
    ) {
        if (A.empty() || B.empty() ||
            A.size() != B.size() ||
            A[0].size() != B[0].size()) {

            flags.sizematch = false;
            cerr << "Matrix size mismatch" << endl;
            return {};
        }

        flags.sizematch = true;

        int rows = A.size();
        int cols = A[0].size();

        vector<vector<int>> C(
            rows,
            vector<int>(cols)
        );

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                C[i][j] = A[i][j] - B[i][j];
            }
        }

        return C;
    }

    vector<vector<int>> matrix_mul(
        const vector<vector<int>>& A,
        const vector<vector<int>>& B
    ) {
        if (A.empty() || B.empty() ||
            A[0].size() != B.size()) {

            flags.sizematch = false;
            cerr << "Matrix size mismatch" << endl;
            return {};
        }

        flags.sizematch = true;

        int rowsA = A.size();
        int colsA = A[0].size();
        int colsB = B[0].size();

        vector<vector<int>> C(
            rowsA,
            vector<int>(colsB, 0)
        );

        for (int i = 0; i < rowsA; i++) {
            for (int j = 0; j < colsB; j++) {
                for (int k = 0; k < colsA; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }

        return C;
    }

    vector<vector<int>> matrix_transpose(
        const vector<vector<int>>& A
    ) {
        if (A.empty()) {
            return {};
        }

        int rows = A.size();
        int cols = A[0].size();

        vector<vector<int>> C(
            cols,
            vector<int>(rows)
        );

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                C[j][i] = A[i][j];
            }
        }

        return C;
    }
};
