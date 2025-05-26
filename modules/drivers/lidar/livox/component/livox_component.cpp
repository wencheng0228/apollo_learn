/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/lidar/livox/component/livox_component.h"

#include <string>
#include <utility>

#include "modules/drivers/lidar/livox/component/livox_dispatcher.h"
namespace apollo {
namespace drivers {
namespace lidar {

void LivoxLidarComponent::BinaryDataProcess(
        const unsigned char* data,
        int data_type,
        int point_size,
        uint64_t pkt_timestamp,
        uint32_t time_interval) {
    if (data_type == kLivoxLidarCartesianCoordinateHighData) {
        AINFO << "high data received, dot num = " << point_size;
        auto* p_point_data
                = reinterpret_cast<const LivoxLidarCartesianHighRawPoint*>(
                        data);

        for (uint32_t i = 0; i < point_size; ++i) {
            PointXYZIT point;
            point.set_x(p_point_data[i].x * 0.001);
            point.set_y(p_point_data[i].y * 0.001);
            point.set_z(p_point_data[i].z * 0.001);
            point.set_intensity(p_point_data[i].reflectivity);
            uint64_t point_timestamp = pkt_timestamp + i * time_interval;
            point.set_timestamp(point_timestamp);
            integral_queue_.push_back(std::move(point));
        }

        uint64_t integral_timestamp_diff = integral_time_ * 1e9;
        int cnt = 0;
        AINFO << "old front t = " << integral_queue_.front().timestamp()
              << " old back t = " << integral_queue_.back().timestamp();
        while (!integral_queue_.empty()
               && integral_queue_.front().timestamp()
                       < integral_queue_.back().timestamp()
                               - integral_timestamp_diff) {
            integral_queue_.pop_front();
            cnt++;
        }
        AINFO << "integral point cnt = " << integral_queue_.size()
              << ", point pop size = " << cnt;
        AINFO << "new front t = " << integral_queue_.front().timestamp()
              << " new back t = " << integral_queue_.back().timestamp();

    } else if (data_type == kLivoxLidarCartesianCoordinateLowData) {
        AINFO << "low data received, dot num = " << point_size;
        auto* p_point_data
                = reinterpret_cast<const LivoxLidarCartesianLowRawPoint*>(data);
        // todo 后增加

    } else if (data_type == kLivoxLidarSphericalCoordinateData) {
        AINFO << "spherical data received, dot num = " << point_size;
        auto* p_point_data
                = reinterpret_cast<const LivoxLidarSpherPoint*>(data);
        // todo 后增加
    }
}

void LivoxLidarComponent::PointCloudCallback(
        uint32_t handle,
        const uint8_t dev_type,
        LivoxLidarEthernetPacket* data,
        void* client_data) {
    if (data == nullptr) {
        return;
    }

    AINFO << boost::format(
                     "point cloud handle: %d, data_num: %d, data_type: %d, "
                     "length: "
                     "%d, frame_counter: %d\n")
                    % handle % data->dot_num % data->data_type % data->length
                    % data->frame_cnt;

    size_t byte_size = GetEthPacketByteSize(data);
    uint64_t pkt_timestamp = GetEthPacketTimestamp(
            data->time_type, data->timestamp, sizeof(data->timestamp));
    uint32_t time_interval = data->time_interval * 100;  // 0.1us -> 100ns

    BinaryDataProcess(
            data->data,
            data->data_type,
            data->dot_num,
            pkt_timestamp,
            time_interval);

    if (byte_size > 0) {
        std::shared_ptr<livox::LivoxScan> scan_message
                = std::make_shared<livox::LivoxScan>();
        scan_message->set_data_type(data->data_type);
        scan_message->set_timestamp(pkt_timestamp);
        auto* data_addr = static_cast<unsigned char*>(data->data);
        scan_message->set_data(data_addr, byte_size);
        scan_message->set_point_size(data->dot_num);
        LidarComponentBase<livox::LivoxScan>::WriteScan(scan_message);
    }

    CheckTimestampAndPublishPointCloud();
}

size_t LivoxLidarComponent::GetEthPacketByteSize(
        LivoxLidarEthernetPacket* data) {
    size_t byte_size = 0;
    if (data == nullptr || data->data == nullptr) {
        return 0;
    }
    switch (data->data_type) {
    case kLivoxLidarCartesianCoordinateHighData:
        byte_size = sizeof(LivoxLidarCartesianHighRawPoint) * data->dot_num;
        break;
    case kLivoxLidarCartesianCoordinateLowData:
        byte_size = sizeof(LivoxLidarCartesianLowRawPoint) * data->dot_num;
        break;
    case kLivoxLidarSphericalCoordinateData:
        byte_size = sizeof(LivoxLidarSpherPoint) * data->dot_num;
        break;
    default:
        byte_size = 0;
        break;
    }
    return byte_size;
}

void LivoxLidarComponent::PreparePointsMsg(PointCloud& msg) {
    msg.set_height(1);
    msg.set_width(msg.point_size() / msg.height());
    msg.set_is_dense(false);
    const auto timestamp
            = msg.point(static_cast<int>(msg.point_size()) - 1).timestamp();
    msg.set_measurement_time(
            GetSecondTimestampFromNanosecondTimestamp(timestamp));

    double lidar_time = GetSecondTimestampFromNanosecondTimestamp(timestamp);
    double diff_time = msg.header().timestamp_sec() - lidar_time;
    if (diff_time > 0.2) {
        AINFO << "timestamp difference too large " << std::fixed
              << std::setprecision(16)
              << "system time: " << msg.header().timestamp_sec()
              << ", lidar time: " << lidar_time << ", diff is:" << diff_time;
    }

    msg.mutable_header()->set_lidar_timestamp(timestamp);
}

bool LivoxLidarComponent::Init() {
    GetProtoConfig(&config_);
    RETURN_VAL_IF(
            !LidarComponentBase<livox::LivoxScan>::InitBase(
                    config_.config_base()),
            false);

    integral_time_ = config_.integral_time();

    if (config_.config_base().source_type()
        == LidarConfigBase_SourceType_ONLINE_LIDAR) {
        if (!config_.has_enable_sdk_console_log()
            || !config_.enable_sdk_console_log()) {
            DisableLivoxSdkConsoleLogger();
        }
        if (!LivoxLidarSdkInit(config_.lidar_config_file_path().c_str())) {
            AERROR << "livox sdk init fail, maybe init by other component";
            //            LivoxLidarSdkUninit();
        }

        std::string lidar_ip = config_.lidar_ip();
        uint32_t lidar_handle = 0;
        if (!LivoxDispatcher::GetLivoxDispatcherInstance().GetHandleFromIP(
                    lidar_ip, lidar_handle)) {
            AERROR << "livox ip address format error, component init fail";
            return false;
        }

        AINFO << "init lidar, handle = " << lidar_handle;

        LivoxDispatcher::GetLivoxDispatcherInstance()
                .RegisterHandleDispatchCallback(
                        lidar_handle,
                        std::bind(
                                &LivoxLidarComponent::PointCloudCallback,
                                this,
                                std::placeholders::_1,
                                std::placeholders::_2,
                                std::placeholders::_3,
                                std::placeholders::_4));

        SetLivoxLidarPointCloudCallBack(
                GlobalPointCloudCallback,
                reinterpret_cast<void*>(
                        &LivoxDispatcher::GetLivoxDispatcherInstance()));

        AINFO << "livox lidar init success";
    }
    return true;
}

void LivoxLidarComponent::ReadScanCallback(
        const std::shared_ptr<livox::LivoxScan>& scan_message) {
    auto data_addr = (unsigned char*)scan_message->data().c_str();
    BinaryDataProcess(
            data_addr,
            scan_message->data_type(),
            scan_message->point_size(),
            scan_message->timestamp(),
            scan_message->time_interval());

    CheckTimestampAndPublishPointCloud();
}

void LivoxLidarComponent::CheckTimestampAndPublishPointCloud() {
    if (!integral_queue_.empty()) {
        uint64_t timestamp_now = cyber::Time::Now().ToNanosecond();
        uint64_t timestamp_dist
                = timestamp_now - last_pointcloud_pub_timestamp_;

        uint64_t tolerable_timestamp = (1e9 / pointcloud_freq_);

        if (timestamp_dist > tolerable_timestamp) {
            auto pcd_frame = LidarComponentBase<
                    livox::LivoxScan>::AllocatePointCloud();

            for (auto it = integral_queue_.begin(); it != integral_queue_.end();
                 ++it) {
                auto point = pcd_frame->add_point();
                point->CopyFrom(*it);
            }

            PreparePointsMsg(*pcd_frame);
            LidarComponentBase<livox::LivoxScan>::WritePointCloud(pcd_frame);
            AINFO << "pcd frame write, publish timestamp = " << timestamp_now;
            last_pointcloud_pub_timestamp_ = timestamp_now;
        }
    }
}

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
