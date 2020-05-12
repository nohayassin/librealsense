// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Intel Corporation. All Rights Reserved.

//#cmake:add-file ../../../src/algo/depth-to-rgb-calibration/*.cpp

#include "d2rgb-common.h"
#include "F9440687.h"
#include <type_traits>


std::string test_dir( char const * data_dir, char const * test )
{
    std::string dir( root_data_dir );
    dir += data_dir;
    dir += '\\';
    dir += test;
    dir += '\\';
    return dir;
}

template< typename T >
std::vector< T > read_bin_file( char const * data_dir, char const * test, char const * bin )
{
    std::string filename = test_dir( data_dir, test );
    filename += "binFiles\\";
    filename += bin;
    filename += ".bin";
    std::fstream f = std::fstream( filename, std::ios::in | std::ios::binary );
    if( ! f )
        throw std::runtime_error( "failed to read file:\n" + filename );
    f.seekg( 0, f.end );
    size_t cb = f.tellg();
    f.seekg( 0, f.beg );
    if( cb % sizeof( T ) )
        throw std::runtime_error( "file size is not a multiple of data size" );
    std::vector< T > vec( cb / sizeof( T ));
    f.read( (char*) vec.data(), cb );
    f.close();
    return vec;
}
template< typename F, typename D >
void print(size_t x, F f, D d)
{
    // bytes will be written to stdout as characters, which we never want... hence '+fx'
    AC_LOG(DEBUG, "... " << x << ": {matlab}" << +f << " != " << +d << "{c++} (exact)");
}

template<>
void print<algo::k_matrix, algo::k_matrix>(size_t x, algo::k_matrix f, algo::k_matrix d)
{
    // bytes will be written to stdout as characters, which we never want... hence '+fx'
    AC_LOG(DEBUG, "... " <<std::setprecision(15)<< x << ": {matlab}" << f.fx <<" "<< f.fy <<" "<< f.ppx << " " << f.ppy << " != " 
        << d.fx << " " << d.fy << " " << d.ppx << " " << d.ppy << "{c++} (exact)");
}

template<>
void print<algo::double2, algo::double2>(size_t x, algo::double2 f, algo::double2 d)
{
    // bytes will be written to stdout as characters, which we never want... hence '+fx'
    AC_LOG(DEBUG, "... " << std::setprecision(15) << x << ": {matlab}" << f.x << " " << f.y << " != "
        << d.x << " " << d.y << "{c++} (exact)");
}
template<>
void print<algo::rotation_in_angles, algo::rotation_in_angles>(size_t x, algo::rotation_in_angles f, algo::rotation_in_angles d)
{
    // bytes will be written to stdout as characters, which we never want... hence '+fx'
    AC_LOG(DEBUG, "... " << std::setprecision(15) << x << ": {matlab}" << f.alpha << " " << f.beta << " " << f.gamma << " != "
        << d.alpha << " " << d.beta << " " << d.gamma << "{c++} (exact)");
}

template<>
void print<algo::p_matrix, algo::p_matrix>(size_t x, algo::p_matrix f, algo::p_matrix d)
{
    std::stringstream s_f;
    std::stringstream s_d;

    for (auto i = 0; i < 12; i++)
    {
        s_f << f.vals[i] << " ";
        s_d << d.vals[i] << " ";
    }

    AC_LOG(DEBUG, "... " << std::setprecision(15) << x << ": {matlab}" << s_f.str() << " != "
        << s_d.str() << "{c++} (exact)");
}

template< typename F, typename D >
bool is_equal_approximetly(F fx, D dx)
{
    return fx == approx(dx);
}

template<>
bool is_equal_approximetly<algo::k_matrix, algo::k_matrix>(algo::k_matrix fx, algo::k_matrix dx)
{
    return fx.fx == approx(dx.fx) && 
           fx.fy == approx(dx.fy) &&
           fx.ppx == approx(dx.ppx) &&
           fx.ppy == approx(dx.ppy);
}

template<>
bool is_equal_approximetly<algo::rotation_in_angles, algo::rotation_in_angles>(algo::rotation_in_angles fx, algo::rotation_in_angles dx)
{
    return fx.alpha == approx(dx.alpha) &&
        fx.beta== approx(dx.beta) &&
        fx.gamma == approx(dx.gamma) ;
}

template<>
bool is_equal_approximetly<algo::p_matrix, algo::p_matrix>(algo::p_matrix fx, algo::p_matrix dx)
{
    for (auto i = 0; i < 12; i++)
    {
        if(fx.vals[i] != approx(dx.vals[i]))
            return false;
    }
    return true;
}

template<>
bool is_equal_approximetly<algo::double2, algo::double2>(algo::double2 f, algo::double2 d)
{
    return f.x == approx(d.x) &&
        f.y == approx(d.y);
}

template< typename F, typename D >
bool compare_same_vectors( std::vector< F > const & matlab, std::vector< D > const & cpp )
{
    assert( matlab.size() == cpp.size() );
    size_t n_mismatches = 0;
    size_t size = matlab.size();
    for( size_t x = 0; x < size; ++x )
    {
        F fx = matlab[x];
        D dx = cpp[x];
        bool const is_comparable = std::numeric_limits< D >::is_exact || std::is_enum< D >::value;
        if( is_comparable )
        {
            if (fx != dx && ++n_mismatches <= 5)
                // bytes will be written to stdout as characters, which we never want... hence '+fx'
                print(x, fx, dx);
        }
        else if(!is_equal_approximetly(fx, dx))
        {
            if( ++n_mismatches <= 5 )
                print(x, fx, dx);
        }
    }
    if( n_mismatches )
        AC_LOG( DEBUG, "... " << n_mismatches << " mismatched values of " << size );
    return (n_mismatches == 0);
}

template<>
bool compare_same_vectors<algo::double2, algo::double2>(std::vector< algo::double2> const & matlab, std::vector< algo::double2> const & cpp)
{
    assert(matlab.size() == cpp.size());
    size_t n_mismatches = 0;
    size_t size = matlab.size();
    auto cpp_vac = cpp;
    auto matlab_vac = matlab;
    std::sort(matlab_vac.begin(), matlab_vac.end(), [](algo::double2 d1, algo::double2 d2) {return d1.x < d2.x; });
    std::sort(cpp_vac.begin(), cpp_vac.end(), [](algo::double2 d1, algo::double2 d2) {return d1.x < d2.x; });
    for (size_t x = 0; x < size; ++x)
    {
        auto fx = matlab_vac[x];
        auto dx = cpp_vac[x];
        bool const is_comparable = std::numeric_limits<double>::is_exact || std::is_enum< double >::value;
        if (is_comparable)
        {
            if (fx != dx && ++n_mismatches <= 5)
                // bytes will be written to stdout as characters, which we never want... hence '+fx'
                AC_LOG(DEBUG, "... " << x << ": {matlab}" << fx.x << " " << fx.y << " != " << dx.x << " " << dx.y << "{c++} (exact)");
        }
        else if (fx.x != approx(dx.x) || fx.y != approx(dx.y))
        {
            if (++n_mismatches <= 5)
                AC_LOG(DEBUG, "... " << x << ": {matlab}" << std::setprecision(12) << fx.x << " " << fx.y << " != " << dx.x << " " << dx.y << "{c++}");
        }
    }
    if (n_mismatches)
        AC_LOG(DEBUG, "... " << n_mismatches << " mismatched values of " << size);
    return (n_mismatches == 0);
}

template<>
bool compare_same_vectors<algo::double3, algo::double3>(std::vector< algo::double3> const & matlab, std::vector< algo::double3> const & cpp)
{
    assert(matlab.size() == cpp.size());
    size_t n_mismatches = 0;
    size_t size = matlab.size();
    auto cpp_vac = cpp;
    auto matlab_vac = matlab;
    std::sort(matlab_vac.begin(), matlab_vac.end(), [](algo::double3 d1, algo::double3 d2) {return d1.x < d2.x; });
    std::sort(cpp_vac.begin(), cpp_vac.end(), [](algo::double3 d1, algo::double3 d2) {return d1.x < d2.x; });
    for (size_t x = 0; x < size; ++x)
    {
        auto fx = matlab_vac[x];
        auto dx = cpp_vac[x];
        bool const is_comparable = std::numeric_limits<double>::is_exact || std::is_enum< double >::value;
        if (is_comparable)
        {
            if (fx != dx && ++n_mismatches <= 5)
                // bytes will be written to stdout as characters, which we never want... hence '+fx'
                AC_LOG(DEBUG, "... " << x << ": {matlab}" << fx.x << " " << fx.y << " " << fx.z << " != " << dx.x << " " << dx.y << " " << dx.z << "{c++} (exact)");
        }
        else if (fx.x != approx(dx.x) || fx.y != approx(dx.y))
        {
            if (++n_mismatches <= 5)
                AC_LOG(DEBUG, "... " << x << ": {matlab}" << std::setprecision(12) << fx.x << " " << fx.y << " " << fx.z << " != " << dx.x << " " << dx.y << " " << dx.z << "{c++}");
        }
    }
    if (n_mismatches)
        AC_LOG(DEBUG, "... " << n_mismatches << " mismatched values of " << size);
    return (n_mismatches == 0);
}

template< typename F, typename D >
std::pair<std::vector< F >, std::vector< D > > sort_vectors(std::vector< F > const & matlab, std::vector< D > const & cpp)
{
    auto f = matlab;
    auto d = cpp;

    std::sort(f.begin(), f.end(), [](F f1, F f2) {return f1 < f2; });
    std::sort(d.begin(), d.end(), [](D d1, D d2) {return d1 < d2; });

    return { f,d };
}

template< typename F, typename D >  // F=in bin; D=in memory
bool compare_to_bin_file(
    std::vector< D > const & vec,
    char const * dir, char const * test, char const * filename,
    size_t width, size_t height,
    //bool( *compare_vectors )(std::vector< F > const &, std::vector< D > const &) = nullptr,
    bool(*compare_vectors)(std::vector< F > const &, std::vector< D > const &) = nullptr,
    std::pair<std::vector< F >, std::vector< D >>(*preproccess_vectors)(std::vector< F > const &, std::vector< D > const &) = nullptr
)
{
    TRACE( "Comparing " << filename << ".bin ..." );
    bool ok = true;
    auto bin = read_bin_file< F >( dir, test, filename );
    if( bin.size() != width * height )
        TRACE( filename << ": {matlab size}" << bin.size() << " != {width}" << width << "x" << height << "{height}" ), ok = false;
    if( vec.size() != bin.size() )
        TRACE( filename << ": {c++ size}" << vec.size() << " != " << bin.size() << "{matlab size}" ), ok = false;
    else
    {
        auto v = vec;
        auto b = bin;
        if (preproccess_vectors)
        {
            auto vecs = preproccess_vectors(bin, vec);
            b = vecs.first;
            v = vecs.second;
        }
        if (compare_vectors && !(*compare_vectors)(b, v))
            ok = false;
        if( !ok )
        {
            //dump_vec( vec, bin, filename, width, height );
            //AC_LOG( DEBUG, "... dump of file written to: " << filename << ".dump" );
        }
    }
    return ok;
}


bool get_calib_from_raw_data(algo::calib& calib, double& cost, char const * dir, char const * test, char const * filename)
{
    auto data_size = sizeof(algo::rotation) +
        sizeof(algo::translation) +
        sizeof(algo::k_matrix) +
        sizeof(algo::p_matrix) +
        3 * sizeof(double) + // alpha, bata, gamma
        sizeof(double); // cost

    auto bin = read_bin_file <double>(dir, test, filename);
    if (bin.size() * sizeof(double) != data_size)
    {
        TRACE(filename << ": {matlab size}" << bin.size() << " != " << data_size);
        return false;
    }

    auto data = bin.data();

    auto k = *(algo::k_matrix*)(data);
    data += sizeof(algo::k_matrix) / sizeof(double);
    auto a = *(double*)(data);
    data += 1;
    auto b = *(double*)(data);
    data += 1;
    auto g = *(double*)(data);
    data += 1;
    auto rotation = *(algo::rotation*)(data);
    data += sizeof(algo::rotation) / sizeof(double);
    auto translation = *(algo::translation*)(data);
    data += sizeof(algo::translation) / sizeof(double);
    auto p_mat = *(algo::p_matrix*)(data);
    data += sizeof(algo::p_matrix) / sizeof(double);
    cost = *(double*)(data);

    calib.k_mat = k;
    calib.rot = rotation;
    calib.trans = translation;
    calib.p_mat = p_mat;
    calib.rot_angles = { a,b,g };

    return true;
}

template< typename D>
bool compare_and_trace(D val_matlab, D val_cpp, std::string const & compared)
{
    if (val_matlab != approx(val_cpp))
    {
        TRACE(compared << " " << val_matlab << " -matlab != " << val_cpp << " -cpp");
        return false;
    }
    return true;
}

bool compare_calib_to_bin_file(
    algo::calib const & calib,
    double cost,
    char const * dir,
    char const * test,
    char const * filename,
    bool gradient = false
)
{
    TRACE( "Comparing " << filename << ".bin ..." );
    algo::calib calib_from_file;
    double cost_matlab;
    auto res = get_calib_from_raw_data(calib_from_file, cost_matlab, dir, test, filename);

    auto intr_matlab = calib_from_file.get_intrinsics();
    auto extr_matlab = calib_from_file.get_extrinsics();
    auto pmat_matlab = calib_from_file.get_p_matrix();

    auto intr_cpp = calib.get_intrinsics();
    auto extr_cpp = calib.get_extrinsics();
    auto pmat_cpp = calib.get_p_matrix();

    bool ok = true;

    ok &= compare_and_trace(cost_matlab, cost, "cost");

    ok &= compare_and_trace(intr_matlab.fx, intr_cpp.fx, "fx");
    ok &= compare_and_trace(intr_matlab.fy, intr_cpp.fy, "fy");
    ok &= compare_and_trace(intr_matlab.ppx, intr_cpp.ppx, "ppx");
    ok &= compare_and_trace(intr_matlab.ppy, intr_cpp.ppy, "ppy");

    if (gradient)
    {
        ok &= compare_and_trace(calib.rot_angles.alpha, calib_from_file.rot_angles.alpha, "alpha");
        ok &= compare_and_trace(calib.rot_angles.beta, calib_from_file.rot_angles.beta, "beta");
        ok &= compare_and_trace(calib.rot_angles.gamma, calib_from_file.rot_angles.gamma, "gamma");
    }
    else
    {
        for (auto i = 0; i < 9; i++)
            ok &= compare_and_trace(extr_matlab.rotation[i], extr_cpp.rotation[i], "rotation[" + std::to_string( i ) + "]");
    }

    for (auto i = 0; i < 3; i++)
        ok &= compare_and_trace(extr_matlab.translation[i], extr_cpp.translation[i], "translation[" + std::to_string( i ) + "]");

    for (auto i = 0; i < 12; i++)
        ok &= compare_and_trace(pmat_matlab.vals[i], pmat_cpp.vals[i], "pmat[" + std::to_string( i ) + "]");

    return ok;
}


std::string generate_file_name( std::string const & prefix, size_t num1, size_t w, size_t h, std::string const & suffix)
{
    return prefix +"_"+ std::to_string(num1)+"_"+ std::to_string(h)+"x"+ std::to_string(w) +"_"+ suffix;
}

std::string generate_file_name(std::string const & prefix, int w, int h, std::string const & suffix)
{
    return prefix + "_" + std::to_string(h) + "x" + std::to_string(w) + "_" + suffix;
}

template< typename T >
void read_binary_file(char const * dir, char const * bin, T * data)
{
    std::string filename = dir;
    filename += bin;
    AC_LOG(DEBUG, "... " << filename);
    std::fstream f = std::fstream(filename, std::ios::in | std::ios::binary);
    if (!f)
        throw std::runtime_error("failed to read file:\n" + filename);
    f.seekg(0, f.end);
    size_t cb = f.tellg();
    f.seekg(0, f.beg);
    if (cb != sizeof(T))
        throw std::runtime_error(to_string()
            << "file size (" << cb << ") does not match data size (" << sizeof(T) << "): " << filename);
    std::vector< T > vec(cb / sizeof(T));
    f.read((char*)data, cb);
    f.close();
}

#define FILE_NAME(PRE, W, H, POST) generate_file_name(PRE, W, H, POST)
#define ITERATION_FILE_NAME(PRE, N, W, H, POST) generate_file_name(PRE, N, W, H, POST)

camera_info read_camera_info(char const * dir)
{
    struct params_bin
    {
        // Some units are supposed to be int but we made matlab write out doubles....
        double depth_width;
        double depth_height;
        double depth_units;
        double k_depth[9];
        double rgb_width;
        double rgb_height;
        double k_rgb[9];
        double coeffs[5];
        double rotation[9];
        double translation[3];
        double p_mat[12];
    };

    params_bin param;
    read_binary_file(test_dir(dir, "2").c_str(), "camera_params.matlab", &param);
    const camera_info ci =
    {
        // RGB
        {
            int(param.rgb_width), int(param.rgb_height),
            float(param.k_rgb[2]),  float(param.k_rgb[5]),
            float(param.k_rgb[0]),  float(param.k_rgb[4]),
            RS2_DISTORTION_BROWN_CONRADY,
            { (float)param.coeffs[0], (float)param.coeffs[1], (float)param.coeffs[2], (float)param.coeffs[3], (float)param.coeffs[4] }
        },
        // Z
        {
            int(param.depth_width), int(param.depth_height),
            float(param.k_depth[2]),  float(param.k_depth[5]),
            float(param.k_depth[0]),  float(param.k_depth[4]),
            RS2_DISTORTION_NONE, {0, 0, 0, 0, 0}
        },
        // EXTRINSICS
        {
            { float(param.rotation[0]), float(param.rotation[1]), float(param.rotation[2]),
              float(param.rotation[3]), float(param.rotation[4]), float(param.rotation[5]),
              float(param.rotation[6]), float(param.rotation[7]), float(param.rotation[8]) },
            { float(param.translation[0]), float(param.translation[1]), float(param.translation[2]) }
        }
    };
    return ci;
}

TEST_CASE("Weights calc", "[d2rgb]")
{
    struct scene_metadata
    {
        int iteration_num = 5;
        double correction_in_pixels = 2.9144;
        int num_of_edges;
        std::string rgb_file;
        std::string rgb_prev_file;
        std::string ir_file;
        std::string z_file;
        
    };

    bool read_calib_from_file = false;

    for (auto dir : data_dirs)
    {
        algo::optimizer cal;

        auto scene = "2";
        scene_metadata md;
        camera_info ci;

        if (read_calib_from_file)
        {
            ci = read_camera_info(dir);
            md = { 4, 2.914122625391939, 5235 , "rgb.raw",
            "rgb.raw",
            "ir.raw",
            "depth.raw" };
        }
        else
        {
           ci = F9440687;

         /*   md = { 14, 2.914122625391939, 3659, "YUY2_YUY2_1920x1080_00.06.13.1372_F9440687_0000.raw",
            "YUY2_YUY2_1920x1080_00.06.13.2368_F9440687_0001.raw",
            "I_GrayScale_1024x768_00.06.13.1484_F9440687_0000.raw",
            "Z_GrayScale_1024x768_00.06.13.1484_F9440687_0000.raw" };*/

            md = { 5, 2.914122625391939, 5089, "YUY2_YUY2_1920x1080_00.00.26.6355_F9440687_0000.raw",
            "YUY2_YUY2_1920x1080_00.00.26.7683_F9440687_0001.raw",
            "I_GrayScale_1024x768_00.00.26.7119_F9440687_0000.raw",
            "Z_GrayScale_1024x768_00.00.26.7119_F9440687_0000.raw" };
        }
        init_algo( cal, test_dir( dir, scene),
            md.rgb_file.c_str(),
            md.rgb_prev_file.c_str(),
            md.ir_file.c_str(),
            md.z_file.c_str(),
            ci);

        auto& z_data = cal.get_z_data();
        auto& ir_data = cal.get_ir_data();
        auto& yuy_data = cal.get_yuy_data();
        auto& depth_data = cal.get_depth_data();

        //---
        auto rgb_h = ci.rgb.height;
        auto rgb_w = ci.rgb.width;
        auto z_h = ci.z.height;
        auto z_w = ci.z.width;
        auto num_of_calib_elements = 32;

        CHECK(compare_to_bin_file< double >(yuy_data.edges, dir, scene, FILE_NAME("YUY2_edge", rgb_w, rgb_h, "double_00").c_str(), rgb_h, rgb_w, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(yuy_data.edges_IDT, dir, scene, FILE_NAME("YUY2_IDT", rgb_w, rgb_h, "double_00").c_str(), rgb_h, rgb_w, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(yuy_data.edges_IDTx, dir, scene, FILE_NAME("YUY2_IDTx", rgb_w, rgb_h, "double_00").c_str(), rgb_h, rgb_w, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(yuy_data.edges_IDTy, dir, scene, FILE_NAME("YUY2_IDTy", rgb_w, rgb_h, "double_00").c_str(), rgb_h, rgb_w, compare_same_vectors));

        ////---
        //CHECK(compare_to_bin_file< double >(ir_data.edges, dir, scene, FILE_NAME("I_edge", z_w, z_h,  "double_00").c_str(), z_h, z_w, compare_same_vectors)); // NOHA :: replaced (V)

        ////---
        //CHECK(compare_to_bin_file< double >(z_data.edges, dir, scene, FILE_NAME("Z_edge", z_w, z_h, "double_00").c_str(), z_h, z_w, compare_same_vectors)); // NOHA :: replaced (V)
        //CHECK(compare_to_bin_file< double >(z_data.supressed_edges, dir, scene, FILE_NAME("Z_edgeSupressed", z_w, z_h, "double_00").c_str(), z_h, z_w, compare_same_vectors)); // NOHA :: replaced (V)
        //CHECK(compare_to_bin_file< byte >(z_data.directions, dir, scene, FILE_NAME("Z_dir", z_w, z_h, "uint8_00").c_str(), z_h, z_w, compare_same_vectors)); // NOHA :: replaced (V)

        //CHECK(compare_to_bin_file< double >(z_data.subpixels_x, dir, scene, FILE_NAME("Z_edgeSubPixel", z_w, z_h, "double_01").c_str(), z_h, z_w, compare_same_vectors)); // NOHA :: replaced (V)
        //CHECK(compare_to_bin_file< double >(z_data.subpixels_y, dir, scene, FILE_NAME("Z_edgeSubPixel", z_w, z_h, "double_00").c_str(), z_h, z_w, compare_same_vectors)); // NOHA :: replaced (V)

        //CHECK(compare_to_bin_file< double >(z_data.weights, dir, scene, FILE_NAME("weightsT", 1, md.num_of_edges,"double_00").c_str(), md.num_of_edges, 1, compare_same_vectors)); // NOHA :: replaced (V)
        //CHECK(compare_to_bin_file< double >(z_data.closest, dir, scene, FILE_NAME("Z_valuesForSubEdges", z_w, z_h, "double_00").c_str(), z_h, z_w, compare_same_vectors)); // NOHA :: replaced (V)
        //CHECK(compare_to_bin_file< algo::double3 >(z_data.vertices, dir, scene, FILE_NAME("vertices", 3, md.num_of_edges, "double_00").c_str(), md.num_of_edges, 1, compare_same_vectors)); // NOHA :: replaced (V)

        // smearing
        CHECK(compare_to_bin_file< double >(depth_data.gradient_x, dir, scene, FILE_NAME("Zx", z_w, z_h, "double_00").c_str(), z_h, z_w, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(depth_data.gradient_y, dir, scene, FILE_NAME("Zy", z_w, z_h, "double_00").c_str(), z_h, z_w, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(ir_data.gradient_x, dir, scene, FILE_NAME("Ix", z_w, z_h, "double_00").c_str(), z_h, z_w, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(ir_data.gradient_y, dir, scene, FILE_NAME("Iy", z_w, z_h, "double_00").c_str(), z_h, z_w, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(ir_data.edges, dir, scene, FILE_NAME("iEdge", z_w, z_h, "double_00").c_str(), z_h, z_w, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(depth_data.edges, dir, scene, FILE_NAME("zedge", z_w, z_h, "double_00").c_str(), z_h, z_w, compare_same_vectors));
        CHECK(compare_to_bin_file< uint8_t >(depth_data.section_map_depth, dir, scene, FILE_NAME("sectionMapDepth",  z_w, z_h, "uint8_00").c_str(), z_h, z_w, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(ir_data.valid_edge_pixels_by_ir, dir, scene, FILE_NAME("validEdgePixelsByIR", z_w, z_h, "double_00").c_str(), z_h, z_w, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(ir_data.valid_location_rc_x, dir, scene, FILE_NAME("gridXValid", 1, 105794, "double_00").c_str(), 105794, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(ir_data.valid_location_rc_y, dir, scene, FILE_NAME("gridYValid", 1, 105794, "double_00").c_str(), 105794, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(ir_data.valid_location_rc, dir, scene, FILE_NAME("locRC", 2, 105794, "double_00").c_str(), 105794, 2, compare_same_vectors));
        CHECK(compare_to_bin_file< uint8_t >(ir_data.valid_section_map, dir, scene, FILE_NAME("sectionMapValid", 1, 105794, "uint8_00").c_str(), 105794, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(ir_data.valid_gradient_x, dir, scene, FILE_NAME("IxValid", 1, 105794, "double_00").c_str(), 105794, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(ir_data.valid_gradient_y, dir, scene, FILE_NAME("IyValid", 1, 105794, "double_00").c_str(), 105794, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(ir_data.direction_deg, dir, scene, FILE_NAME("directionInDeg", 1, 105794, "double_00").c_str(), 105794, 1, compare_same_vectors));
        //CHECK(compare_to_bin_file< double >(ir_data.directions, dir, scene, FILE_NAME("directionIndex", 1, 105794, "double_00").c_str(), 105794, 1, compare_same_vectors)); // it passed, depends on index definition
        CHECK(compare_to_bin_file< double >(ir_data.direction_per_pixel, dir, scene, FILE_NAME("dirPerPixel", 2, 105794, "double_00").c_str(), 105794, 2, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(depth_data.valid_direction_per_pixel, dir, scene, FILE_NAME("validdirPerPixel", 1, 4259, "double_00").c_str(), 4259, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(ir_data.local_region[0], dir, scene, FILE_NAME("localRegion", 2, 105794, "double_00").c_str(), 105794, 2, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(ir_data.local_region[1], dir, scene, FILE_NAME("localRegion", 2, 105794, "double_01").c_str(), 105794, 2, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(ir_data.local_region[2], dir, scene, FILE_NAME("localRegion", 2, 105794, "double_02").c_str(), 105794, 2, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(ir_data.local_region[3], dir, scene, FILE_NAME("localRegion", 2, 105794, "double_03").c_str(), 105794, 2, compare_same_vectors));

        CHECK(compare_to_bin_file< double >(ir_data.local_region_x[0], dir, scene, FILE_NAME("localRegion_x", 1, 105794, "double_00").c_str(), 105794, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(ir_data.local_region_x[1], dir, scene, FILE_NAME("localRegion_x", 1, 105794, "double_01").c_str(), 105794, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(ir_data.local_region_x[2], dir, scene, FILE_NAME("localRegion_x", 1, 105794, "double_02").c_str(), 105794, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(ir_data.local_region_x[3], dir, scene, FILE_NAME("localRegion_x", 1, 105794, "double_03").c_str(), 105794, 1, compare_same_vectors));

        CHECK(compare_to_bin_file< double >(ir_data.local_region_y[0], dir, scene, FILE_NAME("localRegion_y", 1, 105794, "double_00").c_str(), 105794, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(ir_data.local_region_y[1], dir, scene, FILE_NAME("localRegion_y", 1, 105794, "double_01").c_str(), 105794, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(ir_data.local_region_y[2], dir, scene, FILE_NAME("localRegion_y", 1, 105794, "double_02").c_str(), 105794, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(ir_data.local_region_y[3], dir, scene, FILE_NAME("localRegion_y", 1, 105794, "double_03").c_str(), 105794, 1, compare_same_vectors));

        CHECK(compare_to_bin_file< double >(ir_data.local_edges, dir, scene, FILE_NAME("localEdges", 4, 105794, "double_00").c_str(), 105794, 4, compare_same_vectors));
        CHECK(compare_to_bin_file< uint8_t >(ir_data.is_supressed, dir, scene, FILE_NAME("isSupressed", 1, 105794, "uint8_00").c_str(), 105794, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(ir_data.fraq_step, dir, scene, FILE_NAME("fraqStep", 1, 105794, "double_00").c_str(), 105794, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(depth_data.local_rc_subpixel, dir, scene, FILE_NAME("locRCsub", 2, 105794, "double_00").c_str(), 105794, 2, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(depth_data.local_x, dir, scene, FILE_NAME("localZx", 2, 105794, "double_00").c_str(), 105794, 2, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(depth_data.local_y, dir, scene, FILE_NAME("localZy", 2, 105794, "double_00").c_str(), 105794, 2, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(depth_data.gradient, dir, scene, FILE_NAME("zGrad", 2, 105794, "double_00").c_str(), 105794, 2, compare_same_vectors));
        //CHECK(compare_to_bin_file< double >(depth_data.grad_in_direction, dir, scene, FILE_NAME("zGradInDirection", 1, 105794, "double_00").c_str(), 105794, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(depth_data.grad_in_direction, dir, scene, FILE_NAME("validzGradInDirection", 1, 4259, "double_00").c_str(), 4259, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(depth_data.local_values, dir, scene, FILE_NAME("localZvalues", 4, 105794, "double_00").c_str(), 105794, 4, compare_same_vectors));
        //CHECK(compare_to_bin_file< double >(depth_data.values_for_subedges, dir, scene, FILE_NAME("zValuesForSubEdges", 1, 105794, "double_00").c_str(), 105794, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(depth_data.edge_sub_pixel, dir, scene, FILE_NAME("edgeSubPixel", 2, 105794, "double_00").c_str(), 105794, 2, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(depth_data.values_for_subedges, dir, scene, FILE_NAME("validzValuesForSubEdges", 1, 4259, "double_00").c_str(), 4259, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(depth_data.supressed_edges, dir, scene, FILE_NAME("validEdgePixels", 1, 105794, "double_00").c_str(), 105794, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< uint8_t >(depth_data.valid_section_map, dir, scene, FILE_NAME("validsectionMapDepth", 1, 4259, "uint8_00").c_str(), 4259, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(depth_data.valid_directions, dir, scene, FILE_NAME("validdirectionIndex", 1, 4259, "double_00").c_str(), 4259, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(depth_data.valid_edge_sub_pixel, dir, scene, FILE_NAME("validedgeSubPixel", 2, 4259, "double_00").c_str(), 4259, 2, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(depth_data.valid_edge_sub_pixel_x, dir, scene, FILE_NAME("xim", 1, 4259, "double_00").c_str(), 4259, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(depth_data.valid_edge_sub_pixel_y, dir, scene, FILE_NAME("yim", 1, 4259, "double_00").c_str(), 4259, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double>(depth_data.sub_points, dir, scene, FILE_NAME("subPoints", 3, 4259, "double_00").c_str(), 4259, 3, compare_same_vectors));
        CHECK(compare_to_bin_file< algo::double3>(depth_data.vertices_all, dir, scene, FILE_NAME("vertices", 3, 4259, "double_00").c_str(), 4259, 3, compare_same_vectors));
        CHECK(compare_to_bin_file< algo::double2>(depth_data.uvmap, dir, scene, FILE_NAME("uv", 2, 4259, "double_00").c_str(), 4259, 2, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(depth_data.is_inside, dir, scene, FILE_NAME("isInside", 1, 4259, "double_00").c_str(), 4259, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(depth_data.subpixels_x, dir, scene, FILE_NAME("Z_xim", 1, 2506, "double_00").c_str(), 2506, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(depth_data.subpixels_y, dir, scene, FILE_NAME("Z_yim", 1, 2506, "double_00").c_str(), 2506, 1, compare_same_vectors));

        CHECK(compare_to_bin_file< double >(depth_data.closest, dir, scene, FILE_NAME("Z_valuesForSubEdges", 1, 2506, "double_00").c_str(), 2506, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(depth_data.grad_in_direction_inside, dir, scene, FILE_NAME("zGradInDirection", 1, 2506, "double_00").c_str(), 2506, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< double >(depth_data.directions, dir, scene, FILE_NAME("directionIndexInside", 1, 2506, "double_00").c_str(), 2506, 1, compare_same_vectors));
        CHECK(compare_to_bin_file< algo::double3 >(depth_data.vertices, dir, scene, FILE_NAME("vertices", 3, 2506, "double_00").c_str(), 2506, 3, compare_same_vectors));
        CHECK(compare_to_bin_file< uint8_t >(depth_data.section_map_depth_inside, dir, scene, FILE_NAME("sectionMapDepthInside", 1, 2506, "uint8_00").c_str(), 2506, 1, compare_same_vectors));

        // ---
        TRACE( "\nChecking scene validity:" );

        CHECK(!cal.is_scene_valid());

        // edge distribution
        CHECK( compare_to_bin_file< double >( z_data.sum_weights_per_section, dir, scene, FILE_NAME("depthEdgeWeightDistributionPerSectionDepth", 1, 4,"double_00").c_str(), 4, 1, compare_same_vectors ) );
        //CHECK( compare_to_bin_file< byte >( z_data.section_map, dir, scene, FILE_NAME("sectionMapDepth_trans", 1, md.num_of_edges, "uint8_00").c_str(), md.num_of_edges, 1, compare_same_vectors ) ); // NOHA :: replaced by validsectionMapDepth (above in smearing)
        CHECK( compare_to_bin_file< byte >( yuy_data.section_map, dir, scene, FILE_NAME("sectionMapRgb_trans", 1, rgb_w*rgb_h, "uint8_00").c_str(), rgb_w*rgb_h, 1, compare_same_vectors ) );
        CHECK(compare_to_bin_file< double >(yuy_data.sum_weights_per_section, dir, scene, FILE_NAME("edgeWeightDistributionPerSectionRgb", 1, 4, "double_00").c_str(), 4, 1, compare_same_vectors));

        //// gradient balanced
        CHECK(compare_to_bin_file< double >(z_data.sum_weights_per_direction, dir, scene, FILE_NAME("edgeWeightsPerDir", 1, 4, "double_00").c_str(), 4, 1, compare_same_vectors));

        // movment check
        // 1. dilation
        CHECK( compare_to_bin_file< uint8_t >( yuy_data.prev_logic_edges, dir, scene, FILE_NAME("logicEdges", rgb_w, rgb_h, "uint8_00").c_str(), rgb_h, rgb_w, compare_same_vectors ) );
        CHECK( compare_to_bin_file< double >( yuy_data.dilated_image, dir, scene, FILE_NAME("dilatedIm", rgb_w, rgb_h, "double_00").c_str(), rgb_h, rgb_w, compare_same_vectors ) );

        // 2. gausssian
        CHECK( compare_to_bin_file< double >( yuy_data.yuy_diff, dir, scene, FILE_NAME("diffIm_01", rgb_w, rgb_h, "double_00").c_str(), rgb_h, rgb_w, compare_same_vectors ) );
        CHECK( compare_to_bin_file< double >( yuy_data.gaussian_filtered_image, dir, scene, FILE_NAME("diffIm", rgb_w, rgb_h, "double_00").c_str(), rgb_h, rgb_w, compare_same_vectors ) );

        // 3. movemont
        CHECK( compare_to_bin_file< double >( yuy_data.gaussian_diff_masked, dir, scene, FILE_NAME("IDiffMasked", rgb_w, rgb_h, "double_00").c_str(), rgb_h, rgb_w, compare_same_vectors ) );
        CHECK( compare_to_bin_file< uint8_t >( yuy_data.move_suspect, dir, scene, FILE_NAME("ixMoveSuspect", rgb_w, rgb_h, "uint8_00").c_str(), rgb_h, rgb_w, compare_same_vectors ) );


        ////--
        TRACE( "\nOptimizing:" );
        auto cb = [&]( algo::iteration_data_collect const & data )
        {
            auto file = ITERATION_FILE_NAME("calib_iteration", data.iteration + 1, num_of_calib_elements, 1,"double_00");
            CHECK(compare_calib_to_bin_file(data.params.curr_calib, data.params.cost, dir, scene, file.c_str()));

            file = ITERATION_FILE_NAME("uvmap_iteration", data.iteration + 1, 2, md.num_of_edges,"double_00");
            CHECK(compare_to_bin_file< algo::double2 >(data.uvmap, dir, scene, file.c_str(), md.num_of_edges, 1, compare_same_vectors));

            file = ITERATION_FILE_NAME("DVals_iteration", data.iteration + 1, 1, md.num_of_edges, "double_00");
            CHECK(compare_to_bin_file< double >(data.d_vals, dir, scene, file.c_str(), md.num_of_edges, 1, compare_same_vectors, sort_vectors));

            file = ITERATION_FILE_NAME("DxVals_iteration", data.iteration + 1, 1, md.num_of_edges, "double_00");
            CHECK(compare_to_bin_file< double >(data.d_vals_x, dir, scene, file.c_str(), md.num_of_edges, 1, compare_same_vectors, sort_vectors));

            file = ITERATION_FILE_NAME("DyVals_iteration", data.iteration + 1, 1, md.num_of_edges,"double_00");
            CHECK(compare_to_bin_file< double >(data.d_vals_y, dir, scene, file.c_str(), md.num_of_edges, 1, compare_same_vectors, sort_vectors));

           /* file = ITERATION_FILE_NAME("xCoeff_P", data.iteration + 1, sizeof(algo::p_matrix) / sizeof(double), md.num_of_edges, "double_00");
            CHECK(compare_to_bin_file< algo::p_matrix>(data.coeffs_p.x_coeffs, dir, scene, file.c_str(), md.num_of_edges, 1, compare_same_vectors, sort_vectors));*/

            /*file = ITERATION_FILE_NAME("xCoeff_Krgb", data.iteration + 1, sizeof(algo::k_matrix) / sizeof(double), md.num_of_edges, "double_00");
            CHECK(compare_to_bin_file< algo::k_matrix>(data.coeffs_k.x_coeffs, dir, scene, file.c_str(), md.num_of_edges, 1, compare_same_vectors, sort_vectors));

            file = ITERATION_FILE_NAME("yCoeff_Krgb", data.iteration + 1, sizeof(algo::k_matrix) / sizeof(double), md.num_of_edges, "double_00");
            CHECK(compare_to_bin_file< algo::k_matrix>(data.coeffs_k.y_coeffs, dir, scene, file.c_str(), md.num_of_edges, 1, compare_same_vectors, sort_vectors));

            file = ITERATION_FILE_NAME("xCoeff_R", data.iteration + 1, sizeof(algo::rotation_in_angles) / sizeof(double), md.num_of_edges, "double_00");
            CHECK(compare_to_bin_file< algo::rotation_in_angles>(data.coeffs_r.x_coeffs, dir, scene, file.c_str(), md.num_of_edges, 1, compare_same_vectors, sort_vectors));*/

            file = ITERATION_FILE_NAME("grad_iteration", data.iteration + 1, num_of_calib_elements, 1, "double_00");
            CHECK(compare_calib_to_bin_file(data.params.calib_gradients, 0, dir, scene, file.c_str(), true));
        };

        //REQUIRE( cal.optimize( cb ) == md.iteration_num);  // n_iterations

        //auto new_calibration = cal.get_calibration();
        //auto cost = cal.get_cost();

        //CHECK(compare_calib_to_bin_file(new_calibration, cost, dir, scene, FILE_NAME("new_calib", num_of_calib_elements, 1, "double_00").c_str()));


        ////--
        //TRACE( "\nChecking output validity:" );
        //CHECK( ! cal.is_valid_results() );

        ////pixel movement is OK, but some sections have negative cost
        //CHECK( cal.calc_correction_in_pixels() == approx(md.correction_in_pixels));

        //CHECK( compare_to_bin_file< double >( z_data.cost_diff_per_section, dir, scene, FILE_NAME("costDiffPerSection", 4, 1, "double_00").c_str(), 1, 4, compare_same_vectors ) );
     }
}
