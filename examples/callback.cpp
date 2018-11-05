#include <iostream>
#include <iterator>
#include <string>
#include <fstream>
#include <chrono>
#include <functional>

#include <cpd/nonrigid.hpp>
#include <cpd/rigid.hpp>
#include <cpd/gauss_transform_fgt.hpp>
#include <cpd/jsoncpp.hpp>

class CallbackHandler {
public:
    void RigidCallback(const cpd::RigidResult &r) {
        cpd::Matrix matrix = r.matrix();
        size_t iterator = r.iterations;
        auto miliseconds = r.runtime;
        for (uint32_t row_index = 0; row_index < matrix.rows(); ++row_index) {
            for (uint32_t col_index = 0; col_index < matrix.cols(); ++col_index) {
                transformations_.push_back(matrix(row_index, col_index));
            }
        }
        std::cout << cpd::to_json(r) << std::endl;
        milliseconds_after_start_.push_back(miliseconds.count());
    }
    void DumpToFiles(const std::string& output_directory) {
        dump_to_file(transformations_, output_directory + "transformations.bin");
        dump_to_file(milliseconds_after_start_, output_directory + "milliseconds.bin");
    }
private:
    template <class T>
            static void dump_to_file(const std::vector<T>& vec, const std::string& filename) {
        std::ofstream output_file (filename);
        std::ostream_iterator<T> output_iterator(output_file);
        std::copy(vec.begin(), vec.end(), output_iterator);
    }
    std::vector<double> transformations_;
    std::vector<uint32_t> iteration_number_;
    std::vector<uint32_t> milliseconds_after_start_;
};

void NonrigidCallback(const cpd::NonrigidResult &r) {
  std::cout << r.points << std::endl << std::endl;
}

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cout << "Invalid usage: cpd-random <method> <rows> <cols>"
                  << std::endl;
        return 1;
    }
    std::string method = argv[1];
    size_t rows = std::stoi(argv[2]);
    size_t cols = std::stoi(argv[3]);
    cpd::Matrix fixed = cpd::Matrix::Random(rows, cols);
    cpd::Matrix moving = cpd::Matrix::Random(rows, cols);

    if (method == "rigid") {
        cpd::Rigid rigid;
        CallbackHandler handler;
        auto callback = std::bind(&CallbackHandler::RigidCallback, handler, std::placeholders::_1);
        rigid.add_callback(callback);
        auto rigid_result = rigid.run(fixed, moving);
        handler.DumpToFiles("./");
    } else if (method == "rigid_g") {
        cpd::Rigid rigid;
        rigid.gauss_transform(std::move(
                std::unique_ptr<cpd::GaussTransform>(new cpd::GaussTransformFgt())));
        CallbackHandler handler;
        auto callback = std::bind(&CallbackHandler::RigidCallback, handler, std::placeholders::_1);
        rigid.add_callback(callback);
        auto rigid_result = rigid.run(fixed, moving);
        handler.DumpToFiles("./");
    } else {
        std::cout << "Invalid method: " << method << std::endl;
        return 1;
    }
    std::cout << "Registration completed OK" << std::endl;
    return 0;
}


