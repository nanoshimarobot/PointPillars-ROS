/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @author Kosuke Murakami
 * @date 2019/02/26
 */

/**
* @author Yan haixu
* Contact: just github.com/hova88
* @date 2021/04/30
*/

#include "point_pillars.h"

#include <chrono>
#include <iostream>

void PointPillars::InitParams()
{
  // ROS_INFO("YAML config: %s", pp_config_.c_str());
  YAML::Node params = YAML::LoadFile(pp_config_);
  kPillarXSize = params["DATA_CONFIG"]["DATA_PROCESSOR"][2]["VOXEL_SIZE"][0].as<float>();
  kPillarYSize = params["DATA_CONFIG"]["DATA_PROCESSOR"][2]["VOXEL_SIZE"][1].as<float>();
  kPillarZSize = params["DATA_CONFIG"]["DATA_PROCESSOR"][2]["VOXEL_SIZE"][2].as<float>();
  kMinXRange = params["DATA_CONFIG"]["POINT_CLOUD_RANGE"][0].as<float>();
  kMinYRange = params["DATA_CONFIG"]["POINT_CLOUD_RANGE"][1].as<float>();
  kMinZRange = params["DATA_CONFIG"]["POINT_CLOUD_RANGE"][2].as<float>();
  kMaxXRange = params["DATA_CONFIG"]["POINT_CLOUD_RANGE"][3].as<float>();
  kMaxYRange = params["DATA_CONFIG"]["POINT_CLOUD_RANGE"][4].as<float>();
  kMaxZRange = params["DATA_CONFIG"]["POINT_CLOUD_RANGE"][5].as<float>();
  kNumClass = params["CLASS_NAMES"].size();
  kMaxNumPillars =
    params["DATA_CONFIG"]["DATA_PROCESSOR"][2]["MAX_NUMBER_OF_VOXELS"]["test"].as<int>();
  kMaxNumPointsPerPillar =
    params["DATA_CONFIG"]["DATA_PROCESSOR"][2]["MAX_POINTS_PER_VOXEL"].as<int>();
  kInputPointFeature = 5;
  // kMaxNumPoints = kMaxNumPillars * 20;
  std::string kDataSet = params["DATA_CONFIG"]["_BASE_CONFIG_"].as<std::string>();
  std::string str;
  std::string dataSetType;
  str = "nu";
  if (kDataSet.find(str) != std::string::npos) {
    kNumPointFeature = 5;  // [x, y, z, i, 0], nuscenes
    dataSetType = str;
  } else {
    str = "kitti";
    if (kDataSet.find(str) != std::string::npos) {
      kNumPointFeature = 4;  // [x, y, z, 0, 0], kitti
      bool refillDim = params["MODEL"]["VFE"]["WITH_REFILL_DIM"].as<bool>(false);
      if (refillDim) kNumPointFeature += 1;
      dataSetType = str;
    } else {
      std::cout << "DataSet should include 'nuscenes','kitti' in DATA_CONFIG._BASE_CONFIG_"
                << std::endl;
      exit(-1);
    }
  }
  std::cout << "DataSet: " << dataSetType << std::endl;
  VFENetType = 0;
  std::string VFENetTypeStr = params["MODEL"]["VFE"]["NAME"].as<std::string>();
  str = "BoolVFE";
  if (VFENetTypeStr.find(str) != std::string::npos) {
    VFENetType = 1;
  }
  std::cout << "VFE VFENetType: " << VFENetTypeStr << std::endl;

  kNumGatherPointFeature = kNumPointFeature + 6;
  kNumInputBoxFeature = 7;
  kNumOutputBoxFeature =
    params["MODEL"]["DENSE_HEAD"]["TARGET_ASSIGNER_CONFIG"]["BOX_CODER_CONFIG"]["code_size"]
      .as<int>(7);
  kBatchSize = 1;
  kNumIndsForScan = 1024;
  kNumThreads = 64;
  kNumBoxCorners = 8;
  kAnchorStrides = 4;
  kNmsPreMaxsize = params["MODEL"]["POST_PROCESSING"]["NMS_CONFIG"]["NMS_PRE_MAXSIZE"].as<int>();
  kNmsPostMaxsize = params["MODEL"]["POST_PROCESSING"]["NMS_CONFIG"]["NMS_POST_MAXSIZE"].as<int>();
  //params for initialize anchors
  //Adapt to OpenPCDet
  kAnchorNames = params["CLASS_NAMES"].as<std::vector<std::string>>();
  for (int i = 0; i < kAnchorNames.size(); ++i) {
    kAnchorDxSizes.emplace_back(
      params["MODEL"]["DENSE_HEAD"]["ANCHOR_GENERATOR_CONFIG"][i]["anchor_sizes"][0][0]
        .as<float>());
    kAnchorDySizes.emplace_back(
      params["MODEL"]["DENSE_HEAD"]["ANCHOR_GENERATOR_CONFIG"][i]["anchor_sizes"][0][1]
        .as<float>());
    kAnchorDzSizes.emplace_back(
      params["MODEL"]["DENSE_HEAD"]["ANCHOR_GENERATOR_CONFIG"][i]["anchor_sizes"][0][2]
        .as<float>());
    kAnchorBottom.emplace_back(
      params["MODEL"]["DENSE_HEAD"]["ANCHOR_GENERATOR_CONFIG"][i]["anchor_bottom_heights"][0]
        .as<float>());
  }
  kRPNHeadNum = params["MODEL"]["DENSE_HEAD"]["RPN_HEAD_CFGS"].size();
  kRPNHeadSpaceCount = 0;
  for (int idx_head = 0; idx_head < kRPNHeadNum; ++idx_head) {
    int num_cls_per_head =
      params["MODEL"]["DENSE_HEAD"]["RPN_HEAD_CFGS"][idx_head]["HEAD_CLS_NAME"].size();
    kRPNClsPerHead.emplace_back(num_cls_per_head);
    kRPNHeadStride0.emplace_back(kRPNHeadSpaceCount);

    std::vector<int> value;
    for (int i = 0; i < num_cls_per_head; ++i) {
      value.emplace_back(idx_head + i);
      kRPNHeadStride.emplace_back(kRPNHeadSpaceCount);
      kRPNHeadOffset.emplace_back(i);
      kRPNHeadCount.emplace_back(num_cls_per_head);
    }
    kRPNHeadSpaceCount += num_cls_per_head * num_cls_per_head;
    kMultiheadLabelMapping.emplace_back(value);
  }
  // std::cout << kRPNHeadSpaceCount << std::endl;
  // for (int idx_head = 0; idx_head < kRPNHeadNum; ++idx_head){
  //     std::cout << kRPNClsPerHead[idx_head] << " ";
  // }
  // std::cout << std::endl;
  // for (int idx_head = 0; idx_head < kNumClass; ++idx_head){
  //     std::cout << kRPNHeadStride[idx_head] << " ";
  // }
  // std::cout << std::endl;
  // for (int idx_head = 0; idx_head < kNumClass; ++idx_head){
  //     std::cout << kRPNHeadOffset[idx_head] << " ";
  // }
  // std::cout << std::endl;

  // Generate secondary parameters based on above.
  kGridXSize = static_cast<int>((kMaxXRange - kMinXRange) / kPillarXSize);  //512
  kGridYSize = static_cast<int>((kMaxYRange - kMinYRange) / kPillarYSize);  //512
  kGridZSize = static_cast<int>((kMaxZRange - kMinZRange) / kPillarZSize);  //1
  kNumBevFeatures = params["MODEL"]["MAP_TO_BEV"]["NUM_BEV_FEATURES"].as<int>();
  kRpnInputSize = kNumBevFeatures * kGridYSize * kGridXSize;

  kNumAnchorXinds = static_cast<int>(kGridXSize / kAnchorStrides);  //Width
  kNumAnchorYinds = static_cast<int>(kGridYSize / kAnchorStrides);  //Hight
  kNumAnchor = kNumAnchorXinds * kNumAnchorYinds * 2 * kNumClass;   // H * W * Ro * N = 196608

  kNumAnchorPerCls = kNumAnchorXinds * kNumAnchorYinds * 2;  //H * W * Ro = 32768
  kRpnBoxOutputSize = kNumAnchor * kNumOutputBoxFeature;
  kRpnClsOutputSize = kNumAnchor * kNumClass;
  kRpnDirOutputSize = kNumAnchor * 2;
}

PointPillars::PointPillars(
  const float score_threshold, const float nms_overlap_threshold, const bool use_onnx,
  const std::string pfe_file, const std::string backbone_file, const std::string pp_config)
: score_threshold_(score_threshold),
  nms_overlap_threshold_(nms_overlap_threshold),
  use_onnx_(use_onnx),
  pfe_file_(pfe_file),
  backbone_file_(backbone_file),
  pp_config_(pp_config)
{
  InitParams();
  InitTRT(use_onnx_, VFENetType);
  DeviceMemoryMalloc();

  if (VFENetType == 1) {
    boolvfe_cuda_ptr_.reset(new BoolVFECuda(
      kNumThreads, kInputPointFeature, kNumPointFeature, kGridXSize, kGridYSize, kGridZSize,
      kPillarXSize, kPillarYSize, kPillarZSize, kMinXRange, kMinYRange, kMinZRange));
  } else {
    pp_preprocess_points_cuda_ptr_.reset(new PreprocessPointsCuda(
      kNumThreads, kMaxNumPillars, kMaxNumPointsPerPillar, kInputPointFeature, kNumPointFeature,
      kNumGatherPointFeature, kNumIndsForScan, kGridXSize, kGridYSize, kGridZSize, kPillarXSize,
      kPillarYSize, kPillarZSize, kMinXRange, kMinYRange, kMinZRange));

    scatter_cuda_ptr_.reset(new ScatterCuda(kNumThreads, kGridXSize, kGridYSize));
  }

  const float float_min = std::numeric_limits<float>::lowest();
  const float float_max = std::numeric_limits<float>::max();
  postprocess_cuda_ptr_.reset(new PostprocessCuda(
    kNumThreads, float_min, float_max, kNumClass, kNumAnchorPerCls, kMultiheadLabelMapping,
    score_threshold_, nms_overlap_threshold_, kNmsPreMaxsize, kNmsPostMaxsize, kNumBoxCorners,
    kNumInputBoxFeature, kNumOutputBoxFeature, kRPNHeadCount, kRPNHeadStride,
    kRPNHeadOffset)); /*kNumOutputBoxFeature*/
}

void PointPillars::DeviceMemoryMalloc()
{
  bool ret;
  // GPU_CHECK(cudaMalloc(reinterpret_cast<void**>(&dev_points), kMaxNumPoints * kInputPointFeature * sizeof(float))); // in_num_points , 5

  // for pillars
  GPU_CHECK(cudaMalloc(
    reinterpret_cast<void **>(&dev_num_points_per_pillar_), kMaxNumPillars * sizeof(float)));  // M
  GPU_CHECK(
    cudaMalloc(reinterpret_cast<void **>(&dev_x_coors_), kMaxNumPillars * sizeof(int)));  // M
  GPU_CHECK(
    cudaMalloc(reinterpret_cast<void **>(&dev_y_coors_), kMaxNumPillars * sizeof(int)));  // M
  GPU_CHECK(cudaMalloc(
    reinterpret_cast<void **>(&dev_pillar_point_feature_),
    kMaxNumPillars * kMaxNumPointsPerPillar * kNumPointFeature * sizeof(float)));  // [M , m , 4]
  GPU_CHECK(cudaMalloc(
    reinterpret_cast<void **>(&dev_pillar_coors_), kMaxNumPillars * 4 * sizeof(float)));  // [M , 4]
  // for sparse map
  GPU_CHECK(cudaMalloc(
    reinterpret_cast<void **>(&dev_sparse_pillar_map_),
    kNumIndsForScan * kNumIndsForScan * sizeof(int)));  // [1024 , 1024]
  GPU_CHECK(cudaMalloc(
    reinterpret_cast<void **>(&dev_cumsum_along_x_),
    kNumIndsForScan * kNumIndsForScan * sizeof(int)));  // [1024 , 1024]
  GPU_CHECK(cudaMalloc(
    reinterpret_cast<void **>(&dev_cumsum_along_y_),
    kNumIndsForScan * kNumIndsForScan * sizeof(int)));  // [1024 , 1024]

  GPU_CHECK(cudaMalloc(
    reinterpret_cast<void **>(&dev_pfe_gather_feature_),
    kMaxNumPillars * kMaxNumPointsPerPillar * kNumGatherPointFeature * sizeof(float)));
  // std::cout << "dev_pfe_gather_feature_ " << kMaxNumPillars  << " x " << kMaxNumPointsPerPillar  << " x " << kNumGatherPointFeature  << " x " << sizeof(float) << " = "
  //             << kMaxNumPillars * kMaxNumPointsPerPillar * kNumGatherPointFeature * sizeof(float) << std::endl;
  // for trt inference
  // create GPU buffers and a stream

  GPU_CHECK(cudaMalloc(
    &pfe_buffers_[0],
    kMaxNumPillars * kMaxNumPointsPerPillar * kNumGatherPointFeature * sizeof(float)));
  GPU_CHECK(cudaMalloc(&pfe_buffers_[1], kMaxNumPillars * 64 * sizeof(float)));

  // std::cout << "pfe_buffers_[0] " << kMaxNumPillars  << " x " << kMaxNumPointsPerPillar  << " x " << kNumGatherPointFeature  << " x " << sizeof(float) << " = "
  //             << kMaxNumPillars * kMaxNumPointsPerPillar * kNumGatherPointFeature * sizeof(float) << std::endl;
  // std::cout << "pfe_buffers_[1] " << kMaxNumPillars  << " x " << 64  << " x " << sizeof(float) << " = "
  //             << kMaxNumPillars * 64 * sizeof(float) << std::endl;

  // for scatter kernel
  GPU_CHECK(cudaMalloc(
    reinterpret_cast<void **>(&dev_scattered_feature_),
    kNumThreads * kGridYSize * kGridXSize * kGridZSize * sizeof(float)));

  // std::cout << "dev_scattered_feature_ " << kNumThreads  << " x " << kGridYSize  << " x " << kGridXSize  << " x " << sizeof(float) << " = "
  //             << kNumThreads * kGridYSize * kGridXSize * sizeof(float) << std::endl;

  // std::cout << "kRpnInputSize kNumAnchorPerCls kRPNHeadSpaceCount " << kRpnInputSize << " "
  //         << kNumAnchorPerCls << " " << kRPNHeadSpaceCount << std::endl;
  GPU_CHECK(cudaMalloc(&rpn_buffers_[0], kRpnInputSize * sizeof(float)));
  for (int i = 0; i < kRPNHeadNum; ++i) {
    GPU_CHECK(cudaMalloc(
      &rpn_buffers_[i + 1],
      kNumAnchorPerCls * kRPNClsPerHead[i] * kRPNClsPerHead[i] * sizeof(float)));  //classes
  }
  GPU_CHECK(cudaMalloc(
    &rpn_buffers_[kRPNHeadNum + 1],
    kNumAnchorPerCls * kNumClass * kNumOutputBoxFeature * sizeof(float)));  //boxes

  // for filter
  host_box_ = new float[kNumAnchorPerCls * kNumClass * kNumOutputBoxFeature]();
  host_score_ = new float[kNumAnchorPerCls * kRPNHeadSpaceCount]();
  host_filtered_count_ = new int[kNumClass]();
}

PointPillars::~PointPillars()
{
  // GPU_CHECK(cudaFree(dev_points));
  // for pillars
  GPU_CHECK(cudaFree(dev_num_points_per_pillar_));
  GPU_CHECK(cudaFree(dev_x_coors_));
  GPU_CHECK(cudaFree(dev_y_coors_));
  GPU_CHECK(cudaFree(dev_pillar_point_feature_));
  GPU_CHECK(cudaFree(dev_pillar_coors_));
  // for sparse map
  GPU_CHECK(cudaFree(dev_sparse_pillar_map_));
  GPU_CHECK(cudaFree(dev_cumsum_along_x_));
  GPU_CHECK(cudaFree(dev_cumsum_along_y_));
  // for pfe forward
  GPU_CHECK(cudaFree(dev_pfe_gather_feature_));

  GPU_CHECK(cudaFree(pfe_buffers_[0]));
  GPU_CHECK(cudaFree(pfe_buffers_[1]));

  GPU_CHECK(cudaFree(rpn_buffers_[0]));
  for (int i = 0; i < kRPNHeadNum; ++i) {
    GPU_CHECK(cudaFree(rpn_buffers_[i]));
  }
  GPU_CHECK(cudaFree(rpn_buffers_[kRPNHeadNum + 1]));
  pfe_context_->destroy();
  backbone_context_->destroy();
  pfe_engine_->destroy();
  backbone_engine_->destroy();
  // for post process
  GPU_CHECK(cudaFree(dev_scattered_feature_));
  delete[] host_box_;
  delete[] host_score_;
  delete[] host_filtered_count_;
}

void PointPillars::SetDeviceMemoryToZero()
{
  // GPU_CHECK(cudaMemset(dev_points,                 0, kMaxNumPoints * kInputPointFeature * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_num_points_per_pillar_, 0, kMaxNumPillars * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_x_coors_, 0, kMaxNumPillars * sizeof(int)));
  GPU_CHECK(cudaMemset(dev_y_coors_, 0, kMaxNumPillars * sizeof(int)));
  GPU_CHECK(cudaMemset(
    dev_pillar_point_feature_, 0,
    kMaxNumPillars * kMaxNumPointsPerPillar * kNumPointFeature * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_pillar_coors_, 0, kMaxNumPillars * 4 * sizeof(float)));
  GPU_CHECK(cudaMemset(dev_sparse_pillar_map_, 0, kNumIndsForScan * kNumIndsForScan * sizeof(int)));

  GPU_CHECK(cudaMemset(
    dev_pfe_gather_feature_, 0,
    kMaxNumPillars * kMaxNumPointsPerPillar * kNumGatherPointFeature * sizeof(float)));
  GPU_CHECK(cudaMemset(
    pfe_buffers_[0], 0,
    kMaxNumPillars * kMaxNumPointsPerPillar * kNumGatherPointFeature * sizeof(float)));
  GPU_CHECK(cudaMemset(pfe_buffers_[1], 0, kMaxNumPillars * 64 * sizeof(float)));

  GPU_CHECK(cudaMemset(
    dev_scattered_feature_, 0, kNumThreads * kGridYSize * kGridXSize * kGridZSize * sizeof(float)));
}

void PointPillars::InitTRT(const bool use_onnx, const int read_vfe)
{
  if (use_onnx_) {
    // create a TensorRT model from the onnx model and load it into an engine
    OnnxToTRTModel(backbone_file_, &backbone_engine_);
    std::cout << "backbone_engine_ loaded" << std::endl;
    if (read_vfe != 1) {
      OnnxToTRTModel(pfe_file_, &pfe_engine_);
      std::cout << "pfe_file_ loaded" << std::endl;
    }
  } else {
    EngineToTRTModel(backbone_file_, &backbone_engine_);
    if (read_vfe != 1) {
      EngineToTRTModel(pfe_file_, &pfe_engine_);
    }
  }

  //   if (pfe_engine_ == nullptr || backbone_engine_ == nullptr) {
  //       std::cerr << "Failed to load ONNX file.";
  //   }

  // EngineToTRTFile(pfe_file_ + ".trt", &pfe_engine_);
  // EngineToTRTFile(backbone_file_ + ".trt", &backbone_engine_);
  // create execution context from the engine
  if (read_vfe != 1) {
    pfe_context_ = pfe_engine_->createExecutionContext();
  }
  backbone_context_ = backbone_engine_->createExecutionContext();

  //   if (pfe_context_ == nullptr || backbone_context_ == nullptr) {
  //       std::cerr << "Failed to create TensorRT Execution Context.";
  //   }
}

void PointPillars::OnnxToTRTModel(
  const std::string & model_file,  // name of the onnx model
  nvinfer1::ICudaEngine ** engine_ptr)
{
  int verbosity = static_cast<int>(nvinfer1::ILogger::Severity::kWARNING);

  // create the builder
  const auto explicit_batch =
    static_cast<uint32_t>(kBatchSize)
    << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  nvinfer1::IBuilder * builder = nvinfer1::createInferBuilder(g_logger_);
  nvinfer1::INetworkDefinition * network = builder->createNetworkV2(explicit_batch);

  // parse onnx model
  auto parser = nvonnxparser::createParser(*network, g_logger_);
  if (!parser->parseFromFile(model_file.c_str(), verbosity)) {
    std::string msg("failed to parse onnx file");
    g_logger_.log(nvinfer1::ILogger::Severity::kERROR, msg.c_str());
    exit(EXIT_FAILURE);
  }

  // Build the engine
  builder->setMaxBatchSize(kBatchSize);
  // builder->setHalf2Mode(true);
  nvinfer1::IBuilderConfig * config = builder->createBuilderConfig();
  config->setMaxWorkspaceSize(1 << 25);
  nvinfer1::ICudaEngine * engine = builder->buildEngineWithConfig(*network, *config);

  *engine_ptr = engine;
  parser->destroy();
  network->destroy();
  config->destroy();
  builder->destroy();
}

void PointPillars::EngineToTRTFile(
  const std::string & engine_file, nvinfer1::ICudaEngine ** engine_ptr)
{
  nvinfer1::ICudaEngine * engine = *engine_ptr;
  nvinfer1::IHostMemory * serialized_model = engine->serialize();
  std::ofstream ofs(engine_file, std::ios::out | std::ios::binary);
  ofs.write((char *)(serialized_model->data()), serialized_model->size());
  ofs.close();
  engine->destroy();
  serialized_model->destroy();
}

void PointPillars::EngineToTRTModel(
  const std::string & engine_file, nvinfer1::ICudaEngine ** engine_ptr)
{
  int verbosity = static_cast<int>(nvinfer1::ILogger::Severity::kWARNING);
  std::stringstream gieModelStream;
  gieModelStream.seekg(0, gieModelStream.beg);

  std::ifstream cache(engine_file);
  gieModelStream << cache.rdbuf();
  cache.close();
  nvinfer1::IRuntime * runtime = nvinfer1::createInferRuntime(g_logger_);

  if (runtime == nullptr) {
    std::string msg("failed to build runtime parser");
    g_logger_.log(nvinfer1::ILogger::Severity::kERROR, msg.c_str());
    exit(EXIT_FAILURE);
  }
  gieModelStream.seekg(0, std::ios::end);
  const int modelSize = gieModelStream.tellg();

  gieModelStream.seekg(0, std::ios::beg);
  void * modelMem = malloc(modelSize);
  gieModelStream.read((char *)modelMem, modelSize);

  nvinfer1::ICudaEngine * engine = runtime->deserializeCudaEngine(modelMem, modelSize, NULL);
  if (engine == nullptr) {
    std::string msg("failed to build engine parser");
    g_logger_.log(nvinfer1::ILogger::Severity::kERROR, msg.c_str());
    exit(EXIT_FAILURE);
  }
  *engine_ptr = engine;

  std::cout << "Engine File Loaded: " << engine_file << std::endl;
}

void PointPillars::doInference(
  const float * in_points_array, const int in_num_points, std::vector<float> * out_detections,
  std::vector<int> * out_labels, std::vector<float> * out_scores)
{
  if (VFENetType == 0)
    doPPInference(in_points_array, in_num_points, out_detections, out_labels, out_scores);
  else if (VFENetType == 1)
    doBMapInference(in_points_array, in_num_points, out_detections, out_labels, out_scores);
}

void PointPillars::doBMapInference(
  const float * in_points_array, const int in_num_points, std::vector<float> * out_detections,
  std::vector<int> * out_labels, std::vector<float> * out_scores)
{
  auto preprocess_start = std::chrono::high_resolution_clock::now();

  std::cout << "BoolMap inference" << std::endl;
  SetDeviceMemoryToZero();
  cudaDeviceSynchronize();
  // [STEP 1] : load pointcloud
  float * dev_points;
  GPU_CHECK(cudaMalloc(
    reinterpret_cast<void **>(&dev_points),
    in_num_points * kInputPointFeature * sizeof(float)));  // in_num_points , 5
  GPU_CHECK(cudaMemset(dev_points, 0, in_num_points * kInputPointFeature * sizeof(float)));
  GPU_CHECK(cudaMemcpy(
    dev_points, in_points_array, in_num_points * kInputPointFeature * sizeof(float),
    cudaMemcpyHostToDevice));
  // int num_points = in_num_points;
  // if (num_points > kMaxNumPoints) {
  //     std::cout << "Number of Lidar Points " << num_points << " exceed the max number " << kMaxNumPoints << ". Drop the exceeding parts.";
  //     num_points = kMaxNumPoints;
  // }
  // GPU_CHECK(cudaMemcpy(dev_points, in_points_array, num_points * kInputPointFeature * sizeof(float), cudaMemcpyHostToDevice));

  // std::cout << in_num_points  << " " << kInputPointFeature << "\n";
  // for (int i = 0; i < 20; i++){
  //     cout << in_points_array[i] << " ";
  // }
  // cout << "\n";
  // [STEP 2] : preprocess
  host_pillar_count_[0] = 0;
  boolvfe_cuda_ptr_->DoBoolVFECuda(dev_points, in_num_points, dev_scattered_feature_);
  cudaDeviceSynchronize();
  auto preprocess_end = std::chrono::high_resolution_clock::now();
  // DEVICE_SAVE<float>(dev_pfe_gather_feature_,  kMaxNumPillars * kMaxNumPointsPerPillar * kNumGatherPointFeature  , "0_Model_pfe_input_gather_feature");

  // [STEP 5] : backbone forward
  cudaStream_t stream;
  GPU_CHECK(cudaStreamCreate(&stream));
  auto backbone_start = std::chrono::high_resolution_clock::now();
  GPU_CHECK(cudaMemcpyAsync(
    rpn_buffers_[0], dev_scattered_feature_, kBatchSize * kRpnInputSize * sizeof(float),
    cudaMemcpyDeviceToDevice, stream));
  backbone_context_->enqueueV2(rpn_buffers_, stream, nullptr);
  cudaDeviceSynchronize();
  auto backbone_end = std::chrono::high_resolution_clock::now();
  // std::cout << "backbone_context_->enqueueV2" << std::endl;

  // [STEP 6]: postprocess (multihead)
  auto postprocess_start = std::chrono::high_resolution_clock::now();

  // float *temp = new float[kRpnInputSize]();
  // GPU_CHECK(cudaMemcpy(temp, reinterpret_cast<float*>(rpn_buffers_[0]),
  //     kRpnInputSize * sizeof(float), cudaMemcpyDeviceToHost));

  for (int i = 0; i < kRPNHeadNum; i++) {
    GPU_CHECK(cudaMemcpy(
      &host_score_[kRPNHeadStride0[i] * kNumAnchorPerCls],
      reinterpret_cast<float *>(rpn_buffers_[i + 1]),
      kNumAnchorPerCls * kRPNClsPerHead[i] * kRPNClsPerHead[i] * sizeof(float),
      cudaMemcpyDeviceToHost));
  }
  GPU_CHECK(cudaMemcpy(
    host_box_, reinterpret_cast<float *>(rpn_buffers_[kRPNHeadNum + 1]),
    kNumClass * kNumAnchorPerCls * kNumOutputBoxFeature * sizeof(float), cudaMemcpyDeviceToHost));

  // cout << kNumAnchorPerCls << " " << kRPNClsPerHead[0] << " " << kRPNClsPerHead[0] << "\n";
  // for (int i = 100; i < 120; i++){
  //     cout << host_score_[i] << " ";
  // }
  // cout << "\n";
  // cout << kNumClass << " " << kNumAnchorPerCls << " " << kNumOutputBoxFeature << "\n";
  // for (int i = 100; i < 120; i++){
  //     cout << host_box_[i] << " ";
  // }
  // cout << "\n";
  postprocess_cuda_ptr_->DoPostprocessCuda(
    host_box_, host_score_, host_filtered_count_, *out_detections, *out_labels, *out_scores);
  cudaDeviceSynchronize();
  auto postprocess_end = std::chrono::high_resolution_clock::now();

  // release the stream and the buffers
  std::chrono::duration<double> preprocess_cost = preprocess_end - preprocess_start;
  std::chrono::duration<double> backbone_cost = backbone_end - backbone_start;
  std::chrono::duration<double> postprocess_cost = postprocess_end - postprocess_start;

  std::chrono::duration<double> pointpillars_cost = postprocess_end - preprocess_start;
  // std::cout << "------------------------------------" << std::endl;
  // std::cout << setiosflags(ios::left)  << setw(14) << "Module" << setw(12)  << "Time"  << resetiosflags(ios::left) << std::endl;
  // std::cout << "------------------------------------" << std::endl;
  std::string Modules[] = {"Preprocess", "Backbone", "Postprocess", "Summary"};
  double Times[] = {
    preprocess_cost.count(), backbone_cost.count(), postprocess_cost.count(),
    pointpillars_cost.count()};

  for (int i = 0; i < 4; ++i) {
    std::cout << setiosflags(ios::left) << setw(14) << Modules[i] << setw(8) << Times[i] * 1000
              << " ms" << resetiosflags(ios::left);  // << std::endl;
  }
  std::cout << std::endl;
  cudaStreamDestroy(stream);
}

void PointPillars::doPPInference(
  const float * in_points_array, const int in_num_points, std::vector<float> * out_detections,
  std::vector<int> * out_labels, std::vector<float> * out_scores)
{
  std::cout << "Point Pillars inference" << std::endl;
  SetDeviceMemoryToZero();
  cudaDeviceSynchronize();
  // [STEP 1] : load pointcloud
  float * dev_points;
  GPU_CHECK(cudaMalloc(
    reinterpret_cast<void **>(&dev_points),
    in_num_points * kInputPointFeature * sizeof(float)));  // in_num_points , 5
  GPU_CHECK(cudaMemset(dev_points, 0, in_num_points * kInputPointFeature * sizeof(float)));
  GPU_CHECK(cudaMemcpy(
    dev_points, in_points_array, in_num_points * kInputPointFeature * sizeof(float),
    cudaMemcpyHostToDevice));
  // int num_points = in_num_points;
  // if (num_points > kMaxNumPoints) {
  //     std::cout << "Number of Lidar Points " << num_points << " exceed the max number " << kMaxNumPoints << ". Drop the exceeding parts.";
  //     num_points = kMaxNumPoints;
  // }
  // GPU_CHECK(cudaMemcpy(dev_points, in_points_array, num_points * kInputPointFeature * sizeof(float), cudaMemcpyHostToDevice));

  // float * host_temp_space = new float[200000]();
  // GPU_CHECK(cudaMemcpy(host_temp_space, dev_points, 200000 * sizeof(float), cudaMemcpyDeviceToHost));
  // for (int i = 199980; i < 200000; i++)
  //     cout << host_temp_space[i] << " ";
  // cout << std::endl;

  // [STEP 2] : preprocess
  host_pillar_count_[0] = 0;
  auto preprocess_start = std::chrono::high_resolution_clock::now();
  pp_preprocess_points_cuda_ptr_->DoPreprocessPointsCuda(
    dev_points, in_num_points, dev_x_coors_, dev_y_coors_, dev_num_points_per_pillar_,
    dev_pillar_point_feature_, dev_pillar_coors_, dev_sparse_pillar_map_, host_pillar_count_,
    dev_pfe_gather_feature_);
  cudaDeviceSynchronize();
  auto preprocess_end = std::chrono::high_resolution_clock::now();
  // DEVICE_SAVE<float>(
  //   dev_pfe_gather_feature_, kMaxNumPillars * kMaxNumPointsPerPillar * kNumGatherPointFeature,
  //   "0_Model_pfe_input_gather_feature", "/home/toyozoshimada/aist_intern2023_ros2_ws/pillars_log");

  // GPU_CHECK(cudaMemcpy(host_temp_space, dev_pfe_gather_feature_, 200000 * sizeof(float), cudaMemcpyDeviceToHost));
  // for (int i = 199800; i < 200000; i++)
  //     cout << host_temp_space[i] << " ";
  // cout << std::endl;

  // [STEP 3] : pfe forward
  // cudaStream_t stream;
  // GPU_CHECK(cudaStreamCreate(&stream));
  auto pfe_start = std::chrono::high_resolution_clock::now();
  GPU_CHECK(cudaMemcpy(
    pfe_buffers_[0], dev_pfe_gather_feature_,
    kMaxNumPillars * kMaxNumPointsPerPillar * kNumGatherPointFeature * sizeof(float),
    cudaMemcpyDeviceToDevice));
  // GPU_CHECK(cudaMemcpyAsync(
  //   pfe_buffers_[0], dev_pfe_gather_feature_,
  //   kMaxNumPillars * kMaxNumPointsPerPillar * kNumGatherPointFeature *
  //     sizeof(float),  ///kNumGatherPointFeature
  //   cudaMemcpyDeviceToDevice, stream));
  // pfe_context_->enqueueV2(pfe_buffers_, stream, nullptr);
  cudaDeviceSynchronize();
  auto pfe_end = std::chrono::high_resolution_clock::now();
  // DEVICE_SAVE<float>(
  //   reinterpret_cast<float *>(pfe_buffers_[1]), kMaxNumPillars * 64,
  //   "1_Model_pfe_output_buffers_[1]", "/home/toyozoshimada/aist_intern2023_ros2_ws/pillars_log");

  std::cout << "pillar_count " << host_pillar_count_[0] << std::endl;

  // [STEP 4] : scatter pillar feature
  auto scatter_start = std::chrono::high_resolution_clock::now();
  scatter_cuda_ptr_->DoScatterCuda(
    host_pillar_count_[0], dev_x_coors_, dev_y_coors_, reinterpret_cast<float *>(pfe_buffers_[1]),
    dev_scattered_feature_);
  cudaDeviceSynchronize();
  auto scatter_end = std::chrono::high_resolution_clock::now();
  // DEVICE_SAVE<float>(
  //   dev_scattered_feature_, kRpnInputSize, "2_Model_backbone_input_dev_scattered_feature",
  //   "/home/toyozoshimada/aist_intern2023_ros2_ws/pillars_log");

  // GPU_CHECK(cudaMemcpy(host_temp_space, dev_scattered_feature_, 200000 * sizeof(float), cudaMemcpyDeviceToHost));
  // for (int i = 19980; i < 200000; i++)
  //     std::cout << host_temp_space[i] << " ";
  std::cout << "scattered feature size : " << sizeof(dev_scattered_feature_[0]) << std::endl;

  std::cout << "backbone start" << std::endl;

  // [STEP 5] : backbone forward
  auto backbone_start = std::chrono::high_resolution_clock::now();
  // cudaStream_t backbone_stream = nullptr;
  // cudaStreamCreate(&backbone_stream);
  // GPU_CHECK(cudaMemcpyAsync(
  //   rpn_buffers_[0], dev_scattered_feature_, kBatchSize * kRpnInputSize * sizeof(float),
  //   cudaMemcpyDeviceToDevice, backbone_stream));
  // backbone_context_->enqueueV2(rpn_buffers_, backbone_stream, nullptr);
  GPU_CHECK(cudaMemcpy(
    rpn_buffers_[0], dev_scattered_feature_, kBatchSize * kRpnInputSize * sizeof(float),
    cudaMemcpyDeviceToDevice));
  cudaDeviceSynchronize();
  auto backbone_end = std::chrono::high_resolution_clock::now();
  // std::cout << "backbone_context_->enqueueV2" << std::endl;

  // [STEP 6]: postprocess (multihead)
  auto postprocess_start = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < kRPNHeadNum; i++) {
    GPU_CHECK(cudaMemcpy(
      &host_score_[kRPNHeadStride0[i] * kNumAnchorPerCls],
      reinterpret_cast<float *>(rpn_buffers_[i + 1]),
      kNumAnchorPerCls * kRPNClsPerHead[i] * kRPNClsPerHead[i] * sizeof(float),
      cudaMemcpyDeviceToHost));
  }
  GPU_CHECK(cudaMemcpy(
    host_box_, reinterpret_cast<float *>(rpn_buffers_[kRPNHeadNum + 1]),
    kNumClass * kNumAnchorPerCls * kNumOutputBoxFeature * sizeof(float), cudaMemcpyDeviceToHost));

  postprocess_cuda_ptr_->DoPostprocessCuda(
    host_box_, host_score_, host_filtered_count_, *out_detections, *out_labels, *out_scores);
  cudaDeviceSynchronize();
  auto postprocess_end = std::chrono::high_resolution_clock::now();

  // release the stream and the buffers
  std::chrono::duration<double> preprocess_cost = preprocess_end - preprocess_start;
  std::chrono::duration<double> pfe_cost = pfe_end - pfe_start;
  std::chrono::duration<double> scatter_cost = scatter_end - scatter_start;
  std::chrono::duration<double> backbone_cost = backbone_end - backbone_start;
  std::chrono::duration<double> postprocess_cost = postprocess_end - postprocess_start;

  std::chrono::duration<double> pointpillars_cost = postprocess_end - preprocess_start;
  // std::cout << "------------------------------------" << std::endl;
  // std::cout << setiosflags(ios::left)  << setw(14) << "Module" << setw(12)  << "Time"  << resetiosflags(ios::left) << std::endl;
  // std::cout << "------------------------------------" << std::endl;
  std::string Modules[] = {"Preprocess", "Pfe", "Scatter", "Backbone", "Postprocess", "Summary"};
  double Times[] = {preprocess_cost.count(), pfe_cost.count(),         scatter_cost.count(),
                    backbone_cost.count(),   postprocess_cost.count(), pointpillars_cost.count()};

  for (int i = 0; i < 6; ++i) {
    std::cout << setiosflags(ios::left) << setw(14) << Modules[i] << setw(8) << Times[i] * 1000
              << " ms" << resetiosflags(ios::left);  // << std::endl;
  }
  std::cout << std::endl;
  // cudaStreamDestroy(stream);
}
