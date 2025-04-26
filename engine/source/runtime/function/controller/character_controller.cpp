#include "runtime/function/controller/character_controller.h"

#include "runtime/core/base/macro.h"

#include "runtime/function/framework/component/motor/motor_component.h"
#include "runtime/function/framework/world/world_manager.h"
#include "runtime/function/global/global_context.h"
#include "runtime/function/physics/physics_scene.h"

namespace Pilot
{
	CharacterController::CharacterController(const Capsule& capsule) : m_capsule(capsule)
	{
		m_rigidbody_shape = RigidBodyShape();
		m_rigidbody_shape.m_geometry = PILOT_REFLECTION_NEW(Capsule);
		*static_cast<Capsule*>(m_rigidbody_shape.m_geometry) = m_capsule;

		m_rigidbody_shape.m_type = RigidBodyShapeType::capsule;

		Quaternion orientation;
		orientation.fromAngleAxis(Radian(Degree(90.f)), Vector3::UNIT_X);

		m_rigidbody_shape.m_local_transform =
			Transform(
				Vector3(0, 0, capsule.m_half_height + capsule.m_radius),
				orientation,
				Vector3::UNIT_SCALE);
	}

	Vector3 CharacterController::move(const Vector3& current_position, const Vector3& displacement)
	{
		std::shared_ptr<PhysicsScene> physics_scene =
			g_runtime_global_context.m_world_manager->getCurrentActivePhysicsScene().lock();
		ASSERT(physics_scene);

		std::vector<PhysicsHitInfo> hits;

		Transform world_transform = Transform(
			current_position + 0.1f * Vector3::UNIT_Z,
			Quaternion::IDENTITY,
			Vector3::UNIT_SCALE);

		Vector3 vertical_displacement = displacement.z * Vector3::UNIT_Z;
		Vector3 horizontal_displacement = Vector3(displacement.x, displacement.y, 0.f);

		Vector3 vertical_direction = vertical_displacement.normalisedCopy();
		Vector3 horizontal_direction = horizontal_displacement.normalisedCopy();

		Vector3 final_position = current_position;

		m_is_touch_ground = physics_scene->sweep(
			m_rigidbody_shape,
			world_transform.getMatrix(),
			Vector3::NEGATIVE_UNIT_Z,
			0.105f,
			hits);

		hits.clear();

		world_transform.m_position -= 0.1f * Vector3::UNIT_Z;
		int cantMove = 0;
		// vertical pass
		if (physics_scene->sweep(
			m_rigidbody_shape,
			world_transform.getMatrix(),
			vertical_direction,
			vertical_displacement.length(),
			hits))
		{
			final_position += hits[0].hit_distance * vertical_direction;
		}
		else
		{
			final_position += vertical_displacement;
		}

		hits.clear();

		// horizontal pass
		if (physics_scene->sweep(
			m_rigidbody_shape,
			world_transform.getMatrix(),
			horizontal_direction,
			horizontal_displacement.length(),
			hits))
		{
			final_position += hits[0].hit_distance * horizontal_direction;
			if (hits[0].hit_distance <= 0.01) {
				cantMove += 1;
				//LOG_INFO("cant move+1");
			}
		}
		else
		{
			final_position += horizontal_displacement;
		}

		// 沿着墙壁滑动
		// 思路：当无法运动时，找可以移动的方向移动，即运动的方向没有障碍物
		// 又移动的方向应该和输入方向夹角较小，因此扫描移动方向-90~+90范围,从0开始向正负方向扫描，找到较小的偏转角度方向移动
		if (cantMove == 1)
		{
			//LOG_INFO("cant move");
			float originx = horizontal_direction.x;
			float originy = horizontal_direction.y;
			for (float delta = 0; delta <= Math_HALF_PI+ Math_fDeg2Rad; delta += Math_fDeg2Rad)
			{
				float x = cos(delta) * originx - sin(delta) * originy;
				float y = cos(delta) * originy + sin(delta) * originx;
				Vector3 revise_horizontal_direction = Vector3(x, y, 0).normalisedCopy();
				if (!physics_scene->sweep(
					m_rigidbody_shape,
					world_transform.getMatrix(),
					revise_horizontal_direction,
					horizontal_displacement.length(),
					hits))
				{
					final_position += revise_horizontal_direction * horizontal_displacement.length();
					break;
				}

				x = cos(delta) * originx + sin(delta) * originy;
				y = cos(delta) * originy - sin(delta) * originx;
				revise_horizontal_direction = Vector3(x, y, 0).normalisedCopy();
				if (!physics_scene->sweep(
					m_rigidbody_shape,
					world_transform.getMatrix(),
					revise_horizontal_direction,
					horizontal_displacement.length(),
					hits))
				{
					final_position += revise_horizontal_direction * horizontal_displacement.length();
					break;
				}
			}
		}


		int flag = 0;
		//当不在地面上并且卡住时，使用水平方向，前后探测到一个不卡的位置设置为最终位置
		if (!m_is_touch_ground && physics_scene->isOverlap(m_rigidbody_shape, world_transform.getMatrix())) {
			flag = 0;
			for (int i = 1; i < 100; i++)
			{
				if (!physics_scene->sweep(
					m_rigidbody_shape,
					world_transform.getMatrix(),
					-horizontal_direction,
					horizontal_displacement.length() * i,
					hits)) {
					final_position -= horizontal_displacement * i;
					break;
				}
				else if (!physics_scene->sweep(
					m_rigidbody_shape,
					world_transform.getMatrix(),
					horizontal_direction,
					horizontal_displacement.length() * i,
					hits)) {
					final_position += horizontal_displacement * i;
					break;
				}
				flag += 1;
			}
			//std::string str = std::to_string(flag);
			//if (flag == 99)
			//	LOG_INFO("ahha kazhule:");
			//LOG_INFO("aa"+str);
		}


		return final_position;
	}

} // namespace Pilot
