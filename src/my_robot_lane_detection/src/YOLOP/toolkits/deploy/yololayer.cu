#include <assert.h>
#include <cstring>
#include <vector>
#include <iostream>
#include <cstdint>

#include "yololayer.h"
#include "cuda_utils.h"

namespace Tn
{
    template<typename T>
    void write(char*& buffer, T const& val)
    {
        *reinterpret_cast<T*>(buffer) = val;
        buffer += sizeof(T);
    }

    template<typename T>
    void read(char const*& buffer, T& val)
    {
        val = *reinterpret_cast<T const*>(buffer);
        buffer += sizeof(T);
    }
}

using namespace Yolo;

namespace nvinfer1
{
    YoloLayerPlugin::YoloLayerPlugin(
        int classCount,
        int netWidth,
        int netHeight,
        int maxOut,
        std::vector<Yolo::YoloKernel> const& vYoloKernel)
        : mPluginNamespace("")
    {
        mClassCount = classCount;
        mYoloV5NetWidth = netWidth;
        mYoloV5NetHeight = netHeight;
        mMaxOutObject = maxOut;
        mYoloKernel = vYoloKernel;
        mKernelCount = static_cast<int>(vYoloKernel.size());

        CUDA_CHECK(cudaMallocHost(&mAnchor, mKernelCount * sizeof(void*)));
        size_t anchorLen = sizeof(float) * CHECK_COUNT * 2;
        for (int ii = 0; ii < mKernelCount; ++ii)
        {
            CUDA_CHECK(cudaMalloc(&mAnchor[ii], anchorLen));
            auto const& yolo = mYoloKernel[ii];
            CUDA_CHECK(cudaMemcpy(mAnchor[ii], yolo.anchors, anchorLen, cudaMemcpyHostToDevice));
        }
    }

    YoloLayerPlugin::~YoloLayerPlugin()
    {
        if (mAnchor != nullptr)
        {
            for (int ii = 0; ii < mKernelCount; ++ii)
            {
                if (mAnchor[ii] != nullptr)
                {
                    CUDA_CHECK(cudaFree(mAnchor[ii]));
                }
            }
            CUDA_CHECK(cudaFreeHost(mAnchor));
            mAnchor = nullptr;
        }
    }

    // create the plugin at runtime from a byte stream
    YoloLayerPlugin::YoloLayerPlugin(void const* data, size_t length)
        : mPluginNamespace("")
    {
        using namespace Tn;
        char const* d = reinterpret_cast<char const*>(data);
        char const* a = d;

        read(d, mClassCount);
        read(d, mThreadCount);
        read(d, mKernelCount);
        read(d, mYoloV5NetWidth);
        read(d, mYoloV5NetHeight);
        read(d, mMaxOutObject);

        mYoloKernel.resize(mKernelCount);
        auto kernelSize = static_cast<size_t>(mKernelCount) * sizeof(YoloKernel);
        memcpy(mYoloKernel.data(), d, kernelSize);
        d += kernelSize;

        CUDA_CHECK(cudaMallocHost(&mAnchor, mKernelCount * sizeof(void*)));
        size_t anchorLen = sizeof(float) * CHECK_COUNT * 2;
        for (int ii = 0; ii < mKernelCount; ++ii)
        {
            CUDA_CHECK(cudaMalloc(&mAnchor[ii], anchorLen));
            auto const& yolo = mYoloKernel[ii];
            CUDA_CHECK(cudaMemcpy(mAnchor[ii], yolo.anchors, anchorLen, cudaMemcpyHostToDevice));
        }

        assert(d == a + length);
    }

    void YoloLayerPlugin::serialize(void* buffer) const noexcept
    {
        using namespace Tn;
        char* d = static_cast<char*>(buffer);
        char* a = d;

        write(d, mClassCount);
        write(d, mThreadCount);
        write(d, mKernelCount);
        write(d, mYoloV5NetWidth);
        write(d, mYoloV5NetHeight);
        write(d, mMaxOutObject);

        auto kernelSize = static_cast<size_t>(mKernelCount) * sizeof(YoloKernel);
        memcpy(d, mYoloKernel.data(), kernelSize);
        d += kernelSize;

        assert(d == a + getSerializationSize());
    }

    size_t YoloLayerPlugin::getSerializationSize() const noexcept
    {
        return sizeof(mClassCount)
             + sizeof(mThreadCount)
             + sizeof(mKernelCount)
             + sizeof(Yolo::YoloKernel) * mYoloKernel.size()
             + sizeof(mYoloV5NetWidth)
             + sizeof(mYoloV5NetHeight)
             + sizeof(mMaxOutObject);
    }

    int32_t YoloLayerPlugin::initialize() noexcept
    {
        return 0;
    }

    Dims YoloLayerPlugin::getOutputDimensions(int32_t index, Dims const* inputs, int32_t nbInputDims) noexcept
    {
        (void)index;
        (void)inputs;
        (void)nbInputDims;

        int totalsize = mMaxOutObject * static_cast<int>(sizeof(Detection) / sizeof(float));
        return Dims3(totalsize + 1, 1, 1);
    }

    void YoloLayerPlugin::setPluginNamespace(char const* pluginNamespace) noexcept
    {
        mPluginNamespace = (pluginNamespace != nullptr) ? pluginNamespace : "";
    }

    char const* YoloLayerPlugin::getPluginNamespace() const noexcept
    {
        return mPluginNamespace;
    }

    DataType YoloLayerPlugin::getOutputDataType(
        int32_t index,
        DataType const* inputTypes,
        int32_t nbInputs) const noexcept
    {
        (void)index;
        (void)inputTypes;
        (void)nbInputs;
        return DataType::kFLOAT;
    }

    bool YoloLayerPlugin::isOutputBroadcastAcrossBatch(
        int32_t outputIndex,
        bool const* inputIsBroadcasted,
        int32_t nbInputs) const noexcept
    {
        (void)outputIndex;
        (void)inputIsBroadcasted;
        (void)nbInputs;
        return false;
    }

    bool YoloLayerPlugin::canBroadcastInputAcrossBatch(int32_t inputIndex) const noexcept
    {
        (void)inputIndex;
        return false;
    }

    void YoloLayerPlugin::configurePlugin(
        PluginTensorDesc const* in,
        int32_t nbInput,
        PluginTensorDesc const* out,
        int32_t nbOutput) noexcept
    {
        (void)in;
        (void)nbInput;
        (void)out;
        (void)nbOutput;
    }

    void YoloLayerPlugin::attachToContext(
        cudnnContext* cudnnContext,
        cublasContext* cublasContext,
        IGpuAllocator* gpuAllocator) noexcept
    {
        (void)cudnnContext;
        (void)cublasContext;
        (void)gpuAllocator;
    }

    void YoloLayerPlugin::detachFromContext() noexcept
    {
    }

    char const* YoloLayerPlugin::getPluginType() const noexcept
    {
        return "YoloLayer_TRT";
    }

    char const* YoloLayerPlugin::getPluginVersion() const noexcept
    {
        return "1";
    }

    void YoloLayerPlugin::destroy() noexcept
    {
        delete this;
    }

    IPluginV2IOExt* YoloLayerPlugin::clone() const noexcept
    {
        auto* p = new YoloLayerPlugin(
            mClassCount,
            mYoloV5NetWidth,
            mYoloV5NetHeight,
            mMaxOutObject,
            mYoloKernel);
        p->setPluginNamespace(mPluginNamespace);
        return p;
    }

    __device__ float Logist(float data)
    {
        return 1.0f / (1.0f + expf(-data));
    }

    __global__ void CalDetection(
        float const* input,
        float* output,
        int noElements,
        int netwidth,
        int netheight,
        int maxoutobject,
        int yoloWidth,
        int yoloHeight,
        float const anchors[CHECK_COUNT * 2],
        int classes,
        int outputElem)
    {
        int idx = threadIdx.x + blockDim.x * blockIdx.x;
        if (idx >= noElements)
        {
            return;
        }

        int total_grid = yoloWidth * yoloHeight;
        int bnIdx = idx / total_grid;
        idx -= total_grid * bnIdx;

        int info_len_i = 5 + classes;
        float const* curInput = input + bnIdx * (info_len_i * total_grid * CHECK_COUNT);

        for (int k = 0; k < 3; ++k)
        {
            float box_prob = Logist(curInput[idx + k * info_len_i * total_grid + 4 * total_grid]);
            if (box_prob < IGNORE_THRESH)
            {
                continue;
            }

            int class_id = 0;
            float max_cls_prob = 0.0f;
            for (int i = 5; i < info_len_i; ++i)
            {
                float p = Logist(curInput[idx + k * info_len_i * total_grid + i * total_grid]);
                if (p > max_cls_prob)
                {
                    max_cls_prob = p;
                    class_id = i - 5;
                }
            }

            float* res_count = output + bnIdx * outputElem;
            int count = static_cast<int>(atomicAdd(res_count, 1.0f));
            if (count >= maxoutobject)
            {
                return;
            }

            char* data = reinterpret_cast<char*>(res_count) + sizeof(float) + count * sizeof(Detection);
            Detection* det = reinterpret_cast<Detection*>(data);

            int row = idx / yoloWidth;
            int col = idx % yoloWidth;

            det->bbox[0] = (col - 0.5f + 2.0f * Logist(curInput[idx + k * info_len_i * total_grid + 0 * total_grid]))
                         * netwidth / yoloWidth;
            det->bbox[1] = (row - 0.5f + 2.0f * Logist(curInput[idx + k * info_len_i * total_grid + 1 * total_grid]))
                         * netheight / yoloHeight;

            det->bbox[2] = 2.0f * Logist(curInput[idx + k * info_len_i * total_grid + 2 * total_grid]);
            det->bbox[2] = det->bbox[2] * det->bbox[2] * anchors[2 * k];

            det->bbox[3] = 2.0f * Logist(curInput[idx + k * info_len_i * total_grid + 3 * total_grid]);
            det->bbox[3] = det->bbox[3] * det->bbox[3] * anchors[2 * k + 1];

            det->conf = box_prob * max_cls_prob;
            det->class_id = static_cast<float>(class_id);
        }
    }

    void YoloLayerPlugin::forwardGpu(
        float const* const* inputs,
        float* output,
        cudaStream_t stream,
        int batchSize)
    {
        (void)stream;

        int outputElem = 1 + mMaxOutObject * static_cast<int>(sizeof(Detection) / sizeof(float));
        for (int idx = 0; idx < batchSize; ++idx)
        {
            CUDA_CHECK(cudaMemset(output + idx * outputElem, 0, sizeof(float)));
        }

        int numElem = 0;
        for (size_t i = 0; i < mYoloKernel.size(); ++i)
        {
            auto const& yolo = mYoloKernel[i];
            numElem = yolo.width * yolo.height * batchSize;

            int threadCount = mThreadCount;
            if (numElem < threadCount)
            {
                threadCount = numElem;
            }

            CalDetection<<<(numElem + threadCount - 1) / threadCount, threadCount, 0, stream>>>(
                inputs[i],
                output,
                numElem,
                mYoloV5NetWidth,
                mYoloV5NetHeight,
                mMaxOutObject,
                yolo.width,
                yolo.height,
                reinterpret_cast<float const*>(mAnchor[i]),
                mClassCount,
                outputElem);
        }
    }

    int32_t YoloLayerPlugin::enqueue(
        int32_t batchSize,
        void const* const* inputs,
        void* const* outputs,
        void* workspace,
        cudaStream_t stream) noexcept
    {
        (void)workspace;
        forwardGpu(
            reinterpret_cast<float const* const*>(inputs),
            reinterpret_cast<float*>(outputs[0]),
            stream,
            batchSize);
        return 0;
    }

    PluginFieldCollection YoloPluginCreator::mFC{};
    std::vector<PluginField> YoloPluginCreator::mPluginAttributes;

    YoloPluginCreator::YoloPluginCreator()
    {
        mPluginAttributes.clear();
        mFC.nbFields = static_cast<int>(mPluginAttributes.size());
        mFC.fields = mPluginAttributes.data();
    }

    char const* YoloPluginCreator::getPluginName() const noexcept
    {
        return "YoloLayer_TRT";
    }

    char const* YoloPluginCreator::getPluginVersion() const noexcept
    {
        return "1";
    }

    PluginFieldCollection const* YoloPluginCreator::getFieldNames() noexcept
    {
        return &mFC;
    }

    IPluginV2IOExt* YoloPluginCreator::createPlugin(
        char const* name,
        PluginFieldCollection const* fc) noexcept
    {
        (void)name;

        int class_count = -1;
        int input_w = -1;
        int input_h = -1;
        int max_output_object_count = -1;
        std::vector<Yolo::YoloKernel> yolo_kernels(3);

        PluginField const* fields = fc->fields;
        for (int i = 0; i < fc->nbFields; ++i)
        {
            if (strcmp(fields[i].name, "netdata") == 0)
            {
                int const* tmp = static_cast<int const*>(fields[i].data);
                class_count = tmp[0];
                input_w = tmp[1];
                input_h = tmp[2];
                max_output_object_count = tmp[3];
            }
            else if (strstr(fields[i].name, "yolodata") != nullptr)
            {
                int const* tmp = static_cast<int const*>(fields[i].data);
                YoloKernel kernel{};
                kernel.width = tmp[0];
                kernel.height = tmp[1];
                for (int j = 0; j < fields[i].length - 2; ++j)
                {
                    kernel.anchors[j] = static_cast<float>(tmp[j + 2]);
                }
                yolo_kernels[2 - (fields[i].name[8] - '1')] = kernel;
            }
        }

        assert(class_count != -1 && input_w != -1 && input_h != -1 && max_output_object_count != -1);

        auto* obj = new YoloLayerPlugin(
            class_count,
            input_w,
            input_h,
            max_output_object_count,
            yolo_kernels);
        obj->setPluginNamespace(mNamespace.c_str());
        return obj;
    }

    IPluginV2IOExt* YoloPluginCreator::deserializePlugin(
        char const* name,
        void const* serialData,
        size_t serialLength) noexcept
    {
        (void)name;

        auto* obj = new YoloLayerPlugin(serialData, serialLength);
        obj->setPluginNamespace(mNamespace.c_str());
        return obj;
    }
}
