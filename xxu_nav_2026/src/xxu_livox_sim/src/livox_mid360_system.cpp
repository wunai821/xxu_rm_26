#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularAcceleration.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/LinearAcceleration.hh>
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

struct PatternRay
{
  gz::math::Vector3d direction{gz::math::Vector3d::UnitX};
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

std::vector<PatternRay> LoadOfficialScanPattern(
  const std::string &_csvPath)
{
  std::vector<PatternRay> rays;
  if (_csvPath.empty())
  {
    return rays;
  }

  std::ifstream csv(_csvPath);
  if (!csv.good())
  {
    gzerr << "LivoxMid360System could not open scan_mode_csv ["
          << _csvPath << "]; using generated fallback pattern\n";
    return rays;
  }

  std::string line;
  while (std::getline(csv, line))
  {
    if (line.empty() || line.front() == '#')
    {
      continue;
    }

    // The official file is comma separated. Replacing commas with spaces
    // also makes the header fail cleanly without a special-case parser.
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream fields(line);
    double time = 0.0;
    double azimuthDeg = 0.0;
    double zenithDeg = 0.0;
    if (!(fields >> time >> azimuthDeg >> zenithDeg) ||
        !std::isfinite(time) || !std::isfinite(azimuthDeg) ||
        !std::isfinite(zenithDeg))
    {
      continue;
    }

    // Livox's official convention is zenith measured from +Z. Gazebo's
    // ray direction uses elevation from the XY plane, so elevation is
    // pi/2 - zenith (not zenith - pi/2, which mirrors the scan vertically).
    const double azimuth = azimuthDeg * kPi / 180.0;
    const double elevation = kPi / 2.0 - zenithDeg * kPi / 180.0;
    const double cosElevation = std::cos(elevation);
    PatternRay ray;
    ray.direction = gz::math::Vector3d(
      cosElevation * std::cos(azimuth),
      cosElevation * std::sin(azimuth),
      std::sin(elevation));
    if (ray.direction.Length() > std::numeric_limits<double>::epsilon())
    {
      rays.push_back(ray);
    }
  }

  return rays;
}

std::vector<PatternRay> GenerateMid360Pattern(
  const int _samples,
  const double _minVertical,
  const double _maxVertical)
{
  std::vector<PatternRay> rays;
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

    PatternRay ray;
    ray.direction = direction;
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

void SetStamp(
  gz::msgs::IMU &_msg,
  const std::chrono::steady_clock::duration &_time)
{
  const auto ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(_time).count();
  auto *stamp = _msg.mutable_header()->mutable_stamp();
  stamp->set_sec(ns / 1000000000);
  stamp->set_nsec(static_cast<int32_t>(ns % 1000000000));
}

void SetFrameId(gz::msgs::IMU &_msg, const std::string &_frameId)
{
  auto *frame = _msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(_frameId);
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
    this->referenceLink = SdfParam<std::string>(_sdf, "reference_link", "gimbal_link");
    this->topic = SdfParam<std::string>(_sdf, "topic", "/mid360/livox_points_gz");
    this->frameId = SdfParam<std::string>(_sdf, "frame_id", "mid360_link");
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
    this->scanModeCsv = SdfParam<std::string>(_sdf, "scan_mode_csv", "");
    this->rangeNoiseStddev = std::max(0.0,
      SdfParam<double>(_sdf, "range_noise_stddev", 0.02));
    this->noiseSeed = SdfParam<int>(_sdf, "noise_seed", 2026);
    this->noiseRng.seed(static_cast<std::mt19937::result_type>(this->noiseSeed));

    this->scanPattern = LoadOfficialScanPattern(this->scanModeCsv);
    if (this->scanPattern.empty())
    {
      this->scanPattern = GenerateMid360Pattern(
        this->samples, this->minVertical, this->maxVertical);
      gzmsg << "LivoxMid360System using generated fallback scan pattern\n";
    }
    else
    {
      gzmsg << "LivoxMid360System loaded official scan pattern ["
            << this->scanModeCsv << "] rays [" << this->scanPattern.size()
            << "]\n";
    }
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
          << "] rays_per_batch [" << this->raysPerBatch
          << "] range_noise_stddev [" << this->rangeNoiseStddev
          << "] noise_seed [" << this->noiseSeed << "]\n";
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || this->scanPattern.empty() ||
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

      if (_info.paused || this->scanPattern.empty() ||
          this->linkEntity == gz::sim::kNullEntity)
      {
        return;
      }
    }

    if (this->raycastPending)
    {
      return;
    }

    if (this->sensorRays.empty())
    {
      this->BeginScan();
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

    const auto sensorWorldPose = *linkWorldPose * this->sensorPose;
    // RaycastData is attached to this plugin's model entity.  Gazebo defines
    // both its ray endpoints and hit points in that entity's coordinates, not
    // world coordinates.  Convert the sensor pose once per batch and keep the
    // request/result conversion in the same frame.
    const auto entityWorldPose = gz::sim::worldPose(this->entity, _ecm);
    this->lastSensorEntityPose = entityWorldPose.Inverse() *
      sensorWorldPose;
    this->currentBatchStart = this->nextRayIndex;
    const size_t remaining = this->sensorRays.size() - this->currentBatchStart;
    this->currentBatchSize =
      std::min(static_cast<size_t>(this->raysPerBatch), remaining);
    this->lastEntityRays = this->EntityRays(
      this->lastSensorEntityPose, this->currentBatchStart, this->currentBatchSize);

    auto *component =
      _ecm.Component<gz::sim::components::RaycastData>(this->entity);
    if (!component)
    {
      RaycastDataInfo raycastData;
      raycastData.rays = this->lastEntityRays;
      _ecm.CreateComponent(this->entity,
        gz::sim::components::RaycastData(raycastData));
    }
    else
    {
      auto raycastData = component->Data();
      raycastData.rays = this->lastEntityRays;
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

    if (_info.paused || this->scanPattern.empty() ||
        this->sensorRays.empty() || !this->raycastPending)
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

    const auto pointCount = std::min(results.size(), this->lastEntityRays.size());
    // Every point in this batch was raycast from lastSensorEntityPose at
    // lastRaycastRequestTime. Use that exact simulation time instead of
    // inventing a timestamp from the point index. The pose and timestamp must
    // describe the same instant, especially when the gimbal is rotating.
    const double batchTimestampNs = static_cast<double>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      this->lastRaycastRequestTime).count());
    for (size_t i = 0; i < pointCount; ++i)
    {
      const auto &ray = this->lastEntityRays[i];
      const auto &result = results[i];
      if (!std::isfinite(result.point.X()) ||
          !std::isfinite(result.point.Y()) ||
          !std::isfinite(result.point.Z()) ||
          !std::isfinite(result.fraction))
      {
        continue;
      }
      const double range = ray.start.Distance(result.point);
      if (!std::isfinite(range) ||
          range < this->rangeMin || range > this->rangeMax ||
          result.fraction <= 0.0 || result.fraction >= 1.0)
      {
        continue;
      }

      auto localPoint =
        this->lastSensorEntityPose.Rot().RotateVectorReverse(
          result.point - this->lastSensorEntityPose.Pos());

      // The official Livox Gazebo model uses Gaussian range noise. Apply it
      // along the measured ray after the exact collision point is returned.
      // This preserves the scan direction while avoiding artificial lateral
      // displacement and keeps invalid ranges out of the published cloud.
      if (this->rangeNoiseStddev > 0.0)
      {
        const double measuredRange = localPoint.Length();
        const double noisyRange = measuredRange + this->rangeNoiseStddev *
          this->standardNormal(this->noiseRng);
        if (measuredRange <= std::numeric_limits<double>::epsilon() ||
            noisyRange < this->rangeMin || noisyRange > this->rangeMax)
        {
          continue;
        }
        localPoint *= noisyRange / measuredRange;
      }

      const size_t rayIndex = this->currentBatchStart + i;
      AccumulatedPoint point;
      point.point = localPoint;
      point.line = static_cast<uint8_t>(rayIndex % 64);
      point.timestamp = batchTimestampNs;
      this->accumulatedPoints.push_back(point);
    }

    this->nextRayIndex += this->currentBatchSize;
    this->raycastPending = false;

    if (this->nextRayIndex >= this->sensorRays.size())
    {
      this->PublishAccumulatedCloud(_info.simTime);
      this->BeginScan();
    }
  }

private:
  void BeginScan()
  {
    this->sensorRays.resize(static_cast<size_t>(this->samples));
    for (size_t i = 0; i < this->sensorRays.size(); ++i)
    {
      this->sensorRays[i] = this->scanPattern[
        (this->scanPatternCursor + i) % this->scanPattern.size()];
    }
    this->scanPatternCursor = (this->scanPatternCursor + this->sensorRays.size()) %
      this->scanPattern.size();
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

  std::vector<RayInfo> EntityRays(
    const gz::math::Pose3d &_sensorEntityPose,
    const size_t _start,
    const size_t _count) const
  {
    std::vector<RayInfo> entityRays;
    entityRays.reserve(_count);
    const size_t end = std::min(this->sensorRays.size(), _start + _count);
    for (size_t i = _start; i < end; ++i)
    {
      const auto &ray = this->sensorRays[i];
      RayInfo entityRay;
      entityRay.start = _sensorEntityPose.Pos();
      entityRay.end = _sensorEntityPose.CoordPositionAdd(
        ray.direction * this->rangeMax);
      entityRays.push_back(entityRay);
    }
    return entityRays;
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

  std::string referenceLink{"gimbal_link"};
  std::string topic{"/mid360/livox_points_gz"};
  std::string frameId{"mid360_link"};
  std::string scanModeCsv;
  double updateRate{10.0};
  int samples{12000};
  int raysPerBatch{240};
  size_t batchCount{1};
  double rangeMin{0.1};
  double rangeMax{40.0};
  double minVertical{-0.1221730};
  double maxVertical{0.9075712};
  double rangeNoiseStddev{0.02};
  int noiseSeed{2026};
  gz::math::Pose3d sensorPose{gz::math::Pose3d::Zero};
  gz::math::Pose3d lastSensorEntityPose{gz::math::Pose3d::Zero};
  std::vector<PatternRay> scanPattern;
  size_t scanPatternCursor{0};
  std::mt19937 noiseRng{2026};
  std::normal_distribution<double> standardNormal{0.0, 1.0};
  std::vector<PatternRay> sensorRays;
  std::vector<RayInfo> lastEntityRays;
  std::vector<AccumulatedPoint> accumulatedPoints;
  std::chrono::steady_clock::duration lastPubTime{0};
  std::chrono::steady_clock::duration lastRaycastRequestTime{0};
  std::chrono::duration<double> batchPeriod{0.01};
  size_t nextRayIndex{0};
  size_t currentBatchStart{0};
  size_t currentBatchSize{0};
  uint64_t postUpdateCalls{0};
  bool raycastPending{false};
};

// Gazebo Harmonic's built-in IMU sensor can associate a sensor on a fixed
// child link with the model's canonical link. When that link is driven by the
// gimbal joint, its reported linear acceleration contains a false component
// proportional to the model's world spawn position. This is large enough to
// make LIO integrate metres of fake motion. Generate the IMU from the actual
// sensor pose trajectory instead. Differentiating the pose also avoids
// depending on optional acceleration components, which are not enabled by
// every Gazebo physics backend.
class LivoxMid360ImuSystem final
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &/*_ecm*/,
    gz::sim::EventManager &/*_eventMgr*/) override
  {
    this->entity = _entity;
    this->model = gz::sim::Model(_entity);
    this->referenceLink = SdfParam<std::string>(_sdf, "reference_link", "mid360_link");
    this->topic = SdfParam<std::string>(_sdf, "topic", "/imu");
    this->frameId = SdfParam<std::string>(_sdf, "frame_id", "mid360_link");
    this->updateRate = std::max(1.0, SdfParam<double>(_sdf, "update_rate", 100.0));
    this->sensorPose = SdfParam<gz::math::Pose3d>(
      _sdf, "sensor_pose", gz::math::Pose3d::Zero);
    this->angularVelocityNoiseStddev = std::max(0.0,
      SdfParam<double>(_sdf, "angular_velocity_noise_stddev", 0.0001));
    this->linearAccelerationNoiseStddev = std::max(0.0,
      SdfParam<double>(_sdf, "linear_acceleration_noise_stddev", 0.001));
    this->usePhysicsKinematics = SdfParam<bool>(
      _sdf, "use_physics_kinematics", true);
    this->noiseSeed = SdfParam<int>(_sdf, "noise_seed", 2026);
    this->noiseRng.seed(static_cast<std::mt19937::result_type>(this->noiseSeed));
    this->updatePeriod = 1.0 / this->updateRate;
    this->pub = this->node.Advertise<gz::msgs::IMU>(this->topic);
    gzmsg << "LivoxMid360ImuSystem publishing [" << this->topic
          << "] frame [" << this->frameId << "] reference_link ["
          << this->referenceLink << "] update_rate [" << this->updateRate
          << "] physics_kinematics [" << this->usePhysicsKinematics
          << "]\n";
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &/*_info*/,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (this->linkEntity != gz::sim::kNullEntity)
    {
      return;
    }

    this->linkEntity = this->FindReferenceLink(_ecm);
    if (this->linkEntity == gz::sim::kNullEntity)
    {
      return;
    }

    gzmsg << "LivoxMid360ImuSystem found reference link ["
          << this->referenceLink << "]\n";

    if (this->usePhysicsKinematics)
    {
      const gz::sim::Link link(this->linkEntity);
      link.EnableVelocityChecks(_ecm);
      link.EnableAccelerationChecks(_ecm);
      this->physicsChecksEnabled = true;
    }
  }

  void PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || this->linkEntity == gz::sim::kNullEntity)
    {
      return;
    }

    const double now = std::chrono::duration<double>(_info.simTime).count();
    const gz::sim::Link link(this->linkEntity);
    const auto linkPose = link.WorldPose(_ecm);
    if (!linkPose)
    {
      return;
    }

    const auto sensorWorldPose = *linkPose * this->sensorPose;
    if (!this->havePreviousPose)
    {
      this->previousSensorWorldPose = sensorWorldPose;
      this->previousSensorTime = now;
      this->havePreviousPose = true;
      return;
    }

    const double dt = now - this->previousSensorTime;
    if (dt <= 0.0 || dt > 0.1)
    {
      this->previousSensorWorldPose = sensorWorldPose;
      this->previousSensorTime = now;
      this->havePreviousVelocity = false;
      return;
    }

    // The delta quaternion axis is expressed in the previous sensor frame.
    // Convert it to the world frame so the angular velocity and the finite
    // difference of the sensor position share one frame of reference.
    auto deltaRotation = this->previousSensorWorldPose.Rot().Inverse() *
      sensorWorldPose.Rot();
    if (deltaRotation.W() < 0.0)
    {
      deltaRotation = -deltaRotation;
    }
    gz::math::Vector3d deltaAxis;
    double deltaAngle = 0.0;
    deltaRotation.AxisAngle(deltaAxis, deltaAngle);
    const auto angularVelocitySensor = deltaAxis * (deltaAngle / dt);
    const auto angularVelocityWorld = sensorWorldPose.Rot().RotateVector(
      angularVelocitySensor);
    const auto sensorVelocityWorld =
      (sensorWorldPose.Pos() - this->previousSensorWorldPose.Pos()) / dt;

    if (!this->havePreviousVelocity)
    {
      this->previousSensorWorldPose = sensorWorldPose;
      this->previousSensorVelocityWorld = sensorVelocityWorld;
      this->previousSensorAngularVelocityWorld = angularVelocityWorld;
      this->previousSensorTime = now;
      this->havePreviousVelocity = true;
      return;
    }

    auto sensorLinearAccelerationWorld =
      (sensorVelocityWorld - this->previousSensorVelocityWorld) / dt;

    // Prefer Gazebo's physics kinematics over a second finite difference.
    // The latter amplifies the small constraint-solver pose jitter of a
    // rotating fixed child link into acceleration spikes.  Link velocities
    // and accelerations are expressed in world coordinates.  Move the link
    // origin acceleration to the IMU position using the rigid-body equation:
    //   a_sensor = a_origin + alpha x r + omega x (omega x r)
    // If a physics backend does not expose one of these components, retain
    // the finite-difference fallback for that update.
    if (this->usePhysicsKinematics && this->physicsChecksEnabled)
    {
      const auto linkLinearAcceleration = link.WorldLinearAcceleration(_ecm);
      const auto linkAngularVelocity = link.WorldAngularVelocity(_ecm);
      const auto linkAngularAcceleration = link.WorldAngularAcceleration(_ecm);
      if (linkLinearAcceleration && linkAngularVelocity &&
          linkAngularAcceleration)
      {
        const auto leverArmWorld = linkPose->Rot().RotateVector(
          this->sensorPose.Pos());
        sensorLinearAccelerationWorld = *linkLinearAcceleration +
          linkAngularAcceleration->Cross(leverArmWorld) +
          linkAngularVelocity->Cross(
            linkAngularVelocity->Cross(leverArmWorld));
      }
    }
    this->previousSensorWorldPose = sensorWorldPose;
    this->previousSensorVelocityWorld = sensorVelocityWorld;
    this->previousSensorAngularVelocityWorld = angularVelocityWorld;
    this->previousSensorTime = now;

    if (this->havePublished && now < this->nextPublishTime)
    {
      return;
    }
    if (!this->havePublished)
    {
      this->nextPublishTime = now;
    }
    do
    {
      this->nextPublishTime += this->updatePeriod;
    } while (this->nextPublishTime <= now);

    // Accelerometer output is specific force: non-gravitational acceleration
    // expressed in the sensor frame. Gazebo's world acceleration is used here
    // instead of the buggy built-in IMU sensor output.
    const gz::math::Vector3d gravityWorld(0.0, 0.0, -9.81);
    auto specificForce = sensorWorldPose.Rot().RotateVectorReverse(
      sensorLinearAccelerationWorld - gravityWorld);
    auto angularVelocity = sensorWorldPose.Rot().RotateVectorReverse(
      angularVelocityWorld);

    specificForce.X() += this->linearAccelerationNoiseStddev *
      this->standardNormal(this->noiseRng);
    specificForce.Y() += this->linearAccelerationNoiseStddev *
      this->standardNormal(this->noiseRng);
    specificForce.Z() += this->linearAccelerationNoiseStddev *
      this->standardNormal(this->noiseRng);
    angularVelocity.X() += this->angularVelocityNoiseStddev *
      this->standardNormal(this->noiseRng);
    angularVelocity.Y() += this->angularVelocityNoiseStddev *
      this->standardNormal(this->noiseRng);
    angularVelocity.Z() += this->angularVelocityNoiseStddev *
      this->standardNormal(this->noiseRng);

    gz::msgs::IMU msg;
    SetStamp(msg, _info.simTime);
    SetFrameId(msg, this->frameId);
    msg.set_entity_name(this->referenceLink + "::imu_sensor");
    auto *orientation = msg.mutable_orientation();
    orientation->set_x(sensorWorldPose.Rot().X());
    orientation->set_y(sensorWorldPose.Rot().Y());
    orientation->set_z(sensorWorldPose.Rot().Z());
    orientation->set_w(sensorWorldPose.Rot().W());
    auto *angular = msg.mutable_angular_velocity();
    angular->set_x(angularVelocity.X());
    angular->set_y(angularVelocity.Y());
    angular->set_z(angularVelocity.Z());
    auto *linear = msg.mutable_linear_acceleration();
    linear->set_x(specificForce.X());
    linear->set_y(specificForce.Y());
    linear->set_z(specificForce.Z());
    this->pub.Publish(msg);
    this->havePublished = true;
  }

private:
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
  std::string referenceLink{"mid360_link"};
  std::string topic{"/imu"};
  std::string frameId{"mid360_link"};
  double updateRate{100.0};
  double updatePeriod{0.01};
  double angularVelocityNoiseStddev{0.0001};
  double linearAccelerationNoiseStddev{0.001};
  bool usePhysicsKinematics{true};
  bool physicsChecksEnabled{false};
  int noiseSeed{2026};
  gz::math::Pose3d sensorPose{gz::math::Pose3d::Zero};
  gz::math::Pose3d previousSensorWorldPose{gz::math::Pose3d::Zero};
  gz::math::Vector3d previousSensorVelocityWorld{gz::math::Vector3d::Zero};
  gz::math::Vector3d previousSensorAngularVelocityWorld{gz::math::Vector3d::Zero};
  std::mt19937 noiseRng{2026};
  std::normal_distribution<double> standardNormal{0.0, 1.0};
  double nextPublishTime{0.0};
  double previousSensorTime{0.0};
  bool havePublished{false};
  bool havePreviousPose{false};
  bool havePreviousVelocity{false};
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

GZ_ADD_PLUGIN(
  xxu_livox_sim::LivoxMid360ImuSystem,
  gz::sim::System,
  xxu_livox_sim::LivoxMid360ImuSystem::ISystemConfigure,
  xxu_livox_sim::LivoxMid360ImuSystem::ISystemPreUpdate,
  xxu_livox_sim::LivoxMid360ImuSystem::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  xxu_livox_sim::LivoxMid360ImuSystem,
  "xxu_livox_sim::LivoxMid360ImuSystem")
