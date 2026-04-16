#ifndef _YOLO_LAYER_H
#define _YOLO_LAYER_H

#include <vector>
#include <string>
#include <cstdint>
#include "NvInfer.h"

namespace Yolo
{
    static constexpr int CHECK_COUNT = 3;
    static constexpr float IGNORE_THRESH = 0.1f;

    struct YoloKernel
    {
        int width;
        int height;
        float anchors[CHECK_COUNT * 2];
    };

    static constexpr int MAX_OUTPUT_BBOX_COUNT = 1000;
    static constexpr int CLASS_NUM = 13;
    static constexpr int INPUT_H = 384;
    static constexpr int INPUT_W = 640;
    static constexpr int IMG_H = 360;
    static constexpr int IMG_W = 640;
    // static constexpr int INPUT_H = 192;
    // static constexpr int INPUT_W = 320;
    // static constexpr int IMG_H = 180;
    // static constexpr int IMG_W = 320;

    static constexpr int LOCATIONS = 4;

    struct alignas(float) Detection
    {
        // center_x center_y w h
        float bbox[LOCATIONS];
        float conf;      // bbox_conf * cls_conf
        float class_id;
    };
}

namespace nvinfer1
{
    class YoloLayerPlugin : public IPluginV2IOExt
    {
    public:
        YoloLayerPlugin(int classCount, int netWidth, int netHeight, int maxOut,
                        const std::vector<Yolo::YoloKernel>& vYoloKernel);
        YoloLayerPlugin(void const* data, size_t length);
        ~YoloLayerPlugin() override;

        int32_t getNbOutputs() const noexcept override
        {
            return 1;
        }

        Dims getOutputDimensions(int32_t index, Dims const* inputs, int32_t nbInputDims) noexcept override;

        int32_t initialize() noexcept override;

        void terminate() noexcept override {}

        size_t getWorkspaceSize(int32_t maxBatchSize) const noexcept override
        {
            return 0;
        }

        int32_t enqueue(int32_t batchSize,
                        void const* const* inputs,
                        void* const* outputs,
                        void* workspace,
                        cudaStream_t stream) noexcept override;

        size_t getSerializationSize() const noexcept override;

        void serialize(void* buffer) const noexcept override;

        bool supportsFormatCombination(int32_t pos,
                                       PluginTensorDesc const* inOut,
                                       int32_t nbInputs,
                                       int32_t nbOutputs) const noexcept override
        {
            return inOut[pos].format == TensorFormat::kLINEAR &&
                   inOut[pos].type == DataType::kFLOAT;
        }

        char const* getPluginType() const noexcept override;

        char const* getPluginVersion() const noexcept override;

        void destroy() noexcept override;

        IPluginV2IOExt* clone() const noexcept override;

        void setPluginNamespace(char const* pluginNamespace) noexcept override;

        char const* getPluginNamespace() const noexcept override;

        DataType getOutputDataType(int32_t index,
                                   DataType const* inputTypes,
                                   int32_t nbInputs) const noexcept override;

        bool isOutputBroadcastAcrossBatch(int32_t outputIndex,
                                          bool const* inputIsBroadcasted,
                                          int32_t nbInputs) const noexcept override;

        bool canBroadcastInputAcrossBatch(int32_t inputIndex) const noexcept override;

        void attachToContext(cudnnContext* cudnnContext,
                             cublasContext* cublasContext,
                             IGpuAllocator* gpuAllocator) noexcept override;

        void configurePlugin(PluginTensorDesc const* in,
                             int32_t nbInput,
                             PluginTensorDesc const* out,
                             int32_t nbOutput) noexcept override;

        void detachFromContext() noexcept override;

    private:
        void forwardGpu(float const* const* inputs, float* output, cudaStream_t stream, int batchSize = 1);

        int mThreadCount = 256;
        char const* mPluginNamespace;
        int mKernelCount;
        int mClassCount;
        int mYoloV5NetWidth;
        int mYoloV5NetHeight;
        int mMaxOutObject;
        std::vector<Yolo::YoloKernel> mYoloKernel;
        void** mAnchor;
    };

    class YoloPluginCreator : public IPluginCreator
    {
    public:
        YoloPluginCreator();

        ~YoloPluginCreator() override = default;

        char const* getPluginName() const noexcept override;

        char const* getPluginVersion() const noexcept override;

        PluginFieldCollection const* getFieldNames() noexcept override;

        IPluginV2IOExt* createPlugin(char const* name,
                                     PluginFieldCollection const* fc) noexcept override;

        IPluginV2IOExt* deserializePlugin(char const* name,
                                          void const* serialData,
                                          size_t serialLength) noexcept override;

        void setPluginNamespace(char const* libNamespace) noexcept override
        {
            mNamespace = libNamespace;
        }

        char const* getPluginNamespace() const noexcept override
        {
            return mNamespace.c_str();
        }

    private:
        std::string mNamespace;
        static PluginFieldCollection mFC;
        static std::vector<PluginField> mPluginAttributes;
    };

    REGISTER_TENSORRT_PLUGIN(YoloPluginCreator);
}

#endif
