#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/RaycastData.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace xxu_livox_sim
{
namespace
{
using gz::sim::components::RayInfo;
using gz::sim::components::RaycastDataInfo;

constexpr double kPi = 3.14159265358979323846;

struct AccumulatedPoint
{
  gz::math::Vector3d point{gz::math::Vector3d::Zero};
  double timestamp{0.0};
  uint8_t line{0};
};

template<typename T>
T SdfParam(
  const std::shared_ptr<const sdf::Element> &_sdf,
  const std::string &_name,
  const T &_defaultValue)
{
  if (!_sdf)
  {
    return _defaultValue;
  }
  return _sdf->Get<T>(_name, _defaultValue).first;
}

std::vector<RayInfo> GenerateMid360Pattern(
  const int _samples,
  const double _rangeMax,
  const double _minVertical,
  const double _maxVertical)
{
  std::vector<RayInfo> rays;
  rays.reserve(static_cast<size_t>(std::max(1, _samples)));

  // Golden-angle stepping gives a dense non-repeating solid-state LiDAR style
  // pattern without needing the vendor scan CSV at runtime.
  constexpr double goldenAngle = 2.39996322972865332;
  const double verticalSpan = std::max(0.001, _maxVertical - _minVertical);

  for (int i = 0; i < std::max(1, _samples); ++i)
  {
    const double azimuth = std::fmod(static_cast<double>(i) * goldenAngle,
      2.0 * kPi) - kPi;
    const double t = std::fmod(static_cast<double>(i) * 0.6180339887498949, 1.0);
    const double elevation = _minVertical + t * verticalSpan;

    const double cosElevation = std::cos(elevation);
    const gz::math::Vector3d direction(
      cosElevation * std::cos(azimuth),
      cosElevation * std::sin(azimuth),
      std::sin(elevation));

    RayInfo ray;
    ray.start = gz::math::Vector3d::Zero;
    ray.end = direction * _rangeMax;
    rays.push_back(ray);
  }

  return rays;
}

void SetStamp(
  gz::msgs::PointCloudPacked &_msg,
  const std::chrono::steady_clock::duration &_time)
{
  const auto ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(_time).count();
  auto *stamp = _msg.mutable_header()->mutable_stamp();
  stamp->set_sec(ns / 1000000000);
  stamp->set_nsec(static_cast<int32_t>(ns % 1000000000));
}
}  // namespace

class LivoxMid360System final
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/) override
  {
    this->entity = _entity;
    this->model = gz::sim::Model(_entity);
    this->referenceLink = SdfParam<std::string>(_sdf, "reference_link", "radar_link");
    this->topic = SdfParam<std::string>(_sdf, "topic", "/mid360/livox_points_gz");
    this->frameId = SdfParam<std::string>(_sdf, "frame_id", "radar_link");
    this->updateRate = std::max(0.1, SdfParam<double>(_sdf, "update_rate", 10.0));
    this->samples = std::max(1, SdfParam<int>(_sdf, "samples", 12000));
    this->raysPerBatch = std::clamp(
      SdfParam<int>(_sdf, "rays_per_batch", 240), 1, this->samples);
    this->rangeMin = std::max(0.0, SdfParam<double>(_sdf, "range_min", 0.1));
    this->rangeMax = std::max(this->rangeMin + 0.01,
      SdfParam<double>(_sdf, "range_max", 40.0));
    this->minVertical = SdfParam<double>(_sdf, "min_vertical", -0.1221730);
    this->maxVertical = SdfParam<double>(_sdf, "max_vertical", 0.9075712);
    this->sensorPose = SdfParam<gz::math::Pose3d>(
      _sdf, "sensor_pose", gz::math::Pose3d::Zero);

    this->sensorRays = GenerateMid360Pattern(
      this->samples, this->rangeMax, this->minVertical, this->maxVertical);
    this->batchCount =
      static_cast<size_t>((this->samples + this->raysPerBatch - 1) /
      this->raysPerBatch);
    this->batchPeriod = std::chrono::duration<double>(
      (1.0 / this->updateRate) / static_cast<double>(std::max<size_t>(1, this->batchCount)));

    _ecm.CreateComponent(this->entity,
      gz::sim::components::RaycastData(RaycastDataInfo{}));

    this->pub = this->node.Advertise<gz::msgs::PointCloudPacked>(this->topic);
    gzmsg << "LivoxMid360System publishing [" << this->topic
          << "] frame [" << this->frameId << "] reference_link ["
          << this->referenceLink << "] samples [" << this->samples
          << "] rays_per_batch [" << this->raysPerBatch << "]\n";
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || this->sensorRays.empty() ||
        this->linkEntity == gz::sim::kNullEntity)
    {
      if (this->linkEntity == gz::sim::kNullEntity)
      {
        this->linkEntity = this->FindReferenceLink(_ecm);
        if (this->linkEntity != gz::sim::kNullEntity)
        {
          gzmsg << "LivoxMid360System found reference link ["
                << this->referenceLink << "]\n";
        }
      }

      if (_info.paused || this->sensorRays.empty() ||
          this->linkEntity == gz::sim::kNullEntity)
      {
        return;
      }
    }

    if (this->raycastPending)
    {
      return;
    }

    if (this->scanStartTime == std::chrono::steady_clock::duration::zero())
    {
      this->BeginScan(_info.simTime);
    }

    if (this->lastRaycastRequestTime != std::chrono::steady_clock::duration::zero())
    {
      const auto elapsed = _info.simTime - this->lastRaycastRequestTime;
      if (elapsed < this->batchPeriod)
      {
        return;
      }
    }

    const gz::sim::Link link(this->linkEntity);
    const auto linkWorldPose = link.WorldPose(_ecm);
    if (!linkWorldPose)
    {
      return;
    }

    this->lastSensorWorldPose = *linkWorldPose * this->sensorPose;
    this->currentBatchStart = this->nextRayIndex;
    const size_t remaining = this->sensorRays.size() - this->currentBatchStart;
    this->currentBatchSize =
      std::min(static_cast<size_t>(this->raysPerBatch), remaining);
    this->lastWorldRays = this->WorldRays(
      this->lastSensorWorldPose, this->currentBatchStart, this->currentBatchSize);

    auto *component =
      _ecm.Component<gz::sim::components::RaycastData>(this->entity);
    if (!component)
    {
      RaycastDataInfo raycastData;
      raycastData.rays = this->lastWorldRays;
      _ecm.CreateComponent(this->entity,
        gz::sim::components::RaycastData(raycastData));
    }
    else
    {
      auto raycastData = component->Data();
      raycastData.rays = this->lastWorldRays;
      raycastData.results.clear();
      _ecm.SetComponentData<gz::sim::components::RaycastData>(
        this->entity, raycastData);
    }

    this->lastRaycastRequestTime = _info.simTime;
    this->raycastPending = true;
  }

  void PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm) override
  {
    ++this->postUpdateCalls;
    if (this->postUpdateCalls == 1)
    {
      gzmsg << "LivoxMid360System PostUpdate started\n";
    }

    if (_info.paused || this->sensorRays.empty() || !this->raycastPending)
    {
      return;
    }

    const auto *component =
      _ecm.Component<gz::sim::components::RaycastData>(this->entity);
    if (!component)
    {
      return;
    }

    const auto &results = component->Data().results;
    if (results.empty())
    {
      if (this->postUpdateCalls % 300 == 0)
      {
        gzmsg << "LivoxMid360System waiting for raycast results\n";
      }
      return;
    }

    const auto pointCount = std::min(results.size(), this->lastWorldRays.size());
    const double scanPeriod = 1.0 / this->updateRate;
    const auto scanStartNs = static_cast<double>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      this->scanStartTime).count());
    for (size_t i = 0; i < pointCount; ++i)
    {
      const auto &ray = this->lastWorldRays[i];
      const auto &result = results[i];
      const double range = ray.start.Distance(result.point);
      if (range < this->rangeMin || range > this->rangeMax ||
          result.fraction <= 0.0 || result.fraction >= 1.0)
      {
        continue;
      }

      const auto localPoint =
        this->lastSensorWorldPose.Rot().RotateVectorReverse(
          result.point - this->lastSensorWorldPose.Pos());

      const size_t rayIndex = this->currentBatchStart + i;
      AccumulatedPoint point;
      point.point = localPoint;
      point.line = static_cast<uint8_t>(rayIndex % 64);
      point.timestamp = scanStartNs + scanPeriod * 1e9 *
        static_cast<double>(rayIndex) /
        static_cast<double>(std::max<size_t>(1, this->sensorRays.size() - 1));
      this->accumulatedPoints.push_back(point);
    }

    this->nextRayIndex += this->currentBatchSize;
    this->raycastPending = false;

    if (this->nextRayIndex >= this->sensorRays.size())
    {
      this->PublishAccumulatedCloud(_info.simTime);
      this->BeginScan(_info.simTime);
    }
  }

private:
  void BeginScan(const std::chrono::steady_clock::duration &_time)
  {
    this->scanStartTime = _time;
    this->nextRayIndex = 0;
    this->currentBatchStart = 0;
    this->currentBatchSize = 0;
    this->accumulatedPoints.clear();
    this->accumulatedPoints.reserve(this->sensorRays.size());
    this->lastRaycastRequestTime = std::chrono::steady_clock::duration::zero();
  }

  void PublishAccumulatedCloud(const std::chrono::steady_clock::duration &_time)
  {
    gz::msgs::PointCloudPacked msg;
    gz::msgs::InitPointCloudPacked(msg, this->frameId, false, {
      {"xyz", gz::msgs::PointCloudPacked::Field::FLOAT32},
      {"intensity", gz::msgs::PointCloudPacked::Field::FLOAT32},
      {"tag", gz::msgs::PointCloudPacked::Field::UINT8},
      {"line", gz::msgs::PointCloudPacked::Field::UINT8},
      {"timestamp", gz::msgs::PointCloudPacked::Field::FLOAT64},
    });
    SetStamp(msg, _time);

    msg.set_width(static_cast<uint32_t>(this->accumulatedPoints.size()));
    msg.set_height(1);
    msg.set_is_bigendian(false);
    msg.set_is_dense(true);
    msg.set_row_step(msg.point_step() * msg.width());
    msg.mutable_data()->resize(msg.row_step());

    gz::msgs::PointCloudPackedIterator<float> x(msg, "x");
    gz::msgs::PointCloudPackedIterator<float> y(msg, "y");
    gz::msgs::PointCloudPackedIterator<float> z(msg, "z");
    gz::msgs::PointCloudPackedIterator<float> intensity(msg, "intensity");
    gz::msgs::PointCloudPackedIterator<uint8_t> tag(msg, "tag");
    gz::msgs::PointCloudPackedIterator<uint8_t> line(msg, "line");
    gz::msgs::PointCloudPackedIterator<double> timestamp(msg, "timestamp");

    for (const auto &point : this->accumulatedPoints)
    {
      *x = static_cast<float>(point.point.X());
      *y = static_cast<float>(point.point.Y());
      *z = static_cast<float>(point.point.Z());
      *intensity = 100.0F;
      *tag = 0;
      *line = point.line;
      *timestamp = point.timestamp;

      ++x;
      ++y;
      ++z;
      ++intensity;
      ++tag;
      ++line;
      ++timestamp;
    }

    this->pub.Publish(msg);
    this->lastPubTime = _time;
  }

  std::vector<RayInfo> WorldRays(
    const gz::math::Pose3d &_sensorWorldPose,
    const size_t _start,
    const size_t _count) const
  {
    std::vector<RayInfo> worldRays;
    worldRays.reserve(_count);
    const size_t end = std::min(this->sensorRays.size(), _start + _count);
    for (size_t i = _start; i < end; ++i)
    {
      const auto &ray = this->sensorRays[i];
      RayInfo worldRay;
      worldRay.start = _sensorWorldPose.CoordPositionAdd(ray.start);
      worldRay.end = _sensorWorldPose.CoordPositionAdd(ray.end);
      worldRays.push_back(worldRay);
    }
    return worldRays;
  }

  gz::sim::Entity FindReferenceLink(
    const gz::sim::EntityComponentManager &_ecm) const
  {
    const auto modelLink = this->model.LinkByName(_ecm, this->referenceLink);
    if (modelLink != gz::sim::kNullEntity)
    {
      return modelLink;
    }

    gz::sim::Entity found{gz::sim::kNullEntity};
    _ecm.Each<gz::sim::components::Link, gz::sim::components::Name>(
      [&](const gz::sim::Entity &_entity,
          const gz::sim::components::Link *,
          const gz::sim::components::Name *_name) -> bool
      {
        if (_name && _name->Data() == this->referenceLink)
        {
          found = _entity;
          return false;
        }
        return true;
      });

    return found;
  }

  gz::sim::Entity entity{gz::sim::kNullEntity};
  gz::sim::Model model;
  gz::sim::Entity linkEntity{gz::sim::kNullEntity};
  gz::transport::Node node;
  gz::transport::Node::Publisher pub;

  std::string referenceLink{"radar_link"};
  std::string topic{"/mid360/livox_points_gz"};
  std::string frameId{"radar_link"};
  double updateRate{10.0};
  int samples{12000};
  int raysPerBatch{240};
  size_t batchCount{1};
  double rangeMin{0.1};
  double rangeMax{40.0};
  double minVertical{-0.1221730};
  double maxVertical{0.9075712};
  gz::math::Pose3d sensorPose{gz::math::Pose3d::Zero};
  gz::math::Pose3d lastSensorWorldPose{gz::math::Pose3d::Zero};
  std::vector<RayInfo> sensorRays;
  std::vector<RayInfo> lastWorldRays;
  std::vector<AccumulatedPoint> accumulatedPoints;
  std::chrono::steady_clock::duration lastPubTime{0};
  std::chrono::steady_clock::duration lastRaycastRequestTime{0};
  std::chrono::steady_clock::duration scanStartTime{0};
  std::chrono::duration<double> batchPeriod{0.01};
  size_t nextRayIndex{0};
  size_t currentBatchStart{0};
  size_t currentBatchSize{0};
  uint64_t postUpdateCalls{0};
  bool raycastPending{false};
};
}  // namespace xxu_livox_sim

GZ_ADD_PLUGIN(
  xxu_livox_sim::LivoxMid360System,
  gz::sim::System,
  xxu_livox_sim::LivoxMid360System::ISystemConfigure,
  xxu_livox_sim::LivoxMid360System::ISystemPreUpdate,
  xxu_livox_sim::LivoxMid360System::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  xxu_livox_sim::LivoxMid360System,
  "xxu_livox_sim::LivoxMid360System")
