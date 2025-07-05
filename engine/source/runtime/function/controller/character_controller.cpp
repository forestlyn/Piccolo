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

		bool temp_m_is_touch_ground = physics_scene->sweep(
			m_rigidbody_shape,
			world_transform.getMatrix(),
			Vector3::NEGATIVE_UNIT_Z,
			0.105f,
			hits);

		hits.clear();

		world_transform.m_position -= 0.1f * Vector3::UNIT_Z;

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
		float const horizontal_stepup = 1.0f; // 水平移动时的步高

		bool is_down_stair = false;
		bool is_up_stair = false;

		// 平移过程中有障碍物，那么可能可以上楼梯，尝试上楼梯
		if (physics_scene->sweep(
			m_rigidbody_shape,
			world_transform.getMatrix(),
			horizontal_direction,
			horizontal_displacement.length(),
			hits))
		{
			//LOG_DEBUG("CharacterController::move: horizontal pass hit, trying to step up");
			Transform step_transform = Transform(
				current_position + horizontal_displacement + horizontal_stepup * Vector3::UNIT_Z,
				Quaternion::IDENTITY,
				Vector3::UNIT_SCALE);
			//先判断最大步高的情况，如果还是有障碍物重合，则不能上楼梯，其实不是很严谨
			if (!physics_scene->isOverlap(m_rigidbody_shape, step_transform.getMatrix()))
			{
				// 找到碰到地面的高度
				for (float i = 0.0f; i <= horizontal_stepup * 2; i += 0.01f)
				{
					if (physics_scene->sweep(
						m_rigidbody_shape,
						step_transform.getMatrix(),
						Vector3::NEGATIVE_UNIT_Z,
						abs(i),
						hits))
					{
						LOG_DEBUG("CharacterController::move: step up success, i = {}", i);
						final_position += horizontal_displacement + Vector3::UNIT_Z * (horizontal_stepup - i);
						is_up_stair = true;
						break;
					}
				}
			}
			if (!is_up_stair)
			{
				// 如果没有上楼梯，那么就平移到障碍物的边缘
				final_position += hits[0].hit_distance * horizontal_direction;
				// 如果碰撞距离小于0.01f，说明平移到不可跨越障碍物边缘了，准备沿着障碍物滑动
				if (hits[0].hit_distance <= 0.01f)
				{
					LOG_DEBUG("CharacterController::move: horizontal pass hit, sliding along wall");
					// 沿着墙壁滑动
					// 思路：当无法运动也不是上楼梯时，找可以移动的方向移动，即运动的方向没有障碍物
					// 又移动的方向应该和输入方向夹角较小，因此扫描移动方向-90~+90范围,从0开始向正负方向扫描，找到较小的偏转角度方向移动
					float originx = horizontal_direction.x;
					float originy = horizontal_direction.y;
					for (float delta = 0; delta <= Math_HALF_PI + Math_fDeg2Rad; delta += Math_fDeg2Rad)
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
			}
		}
		// 平移过程中没有障碍物，那么直接寻找能否在范围内碰到地面，即下楼梯
		else
		{
			// 当在地面上且竖直方向没有跳跃时，才考虑下楼梯，否则会跳不起来
			if (temp_m_is_touch_ground && vertical_displacement.length() <= 0.01f)
			{
				//LOG_DEBUG("CharacterController::move: horizontal pass no hit, trying to step down");
				Transform step_transform = Transform(
					current_position + horizontal_displacement,
					Quaternion::IDENTITY,
					Vector3::UNIT_SCALE);
				// 找到碰到地面的高度
				if (physics_scene->sweep(
					m_rigidbody_shape,
					step_transform.getMatrix(),
					Vector3::NEGATIVE_UNIT_Z,
					abs(horizontal_stepup),
					hits))
				{
					Vector3 downward_hit_displacement = horizontal_displacement + Vector3::NEGATIVE_UNIT_Z * hits[0].hit_distance;
					Vector3 downward_hit_direction = downward_hit_displacement.normalisedCopy();

					// 找到后再扫描移动过去
					if (physics_scene->sweep(
						m_rigidbody_shape,
						world_transform.getMatrix(),
						downward_hit_direction,
						downward_hit_displacement.length(),
						hits))
					{
						if (hits[0].hit_distance >= 0.01f)
						{
							LOG_DEBUG("CharacterController::move: step down success, i = {}", hits[0].hit_distance);
							final_position += downward_hit_direction * hits[0].hit_distance;
							is_down_stair = true;
						}
					}
				}
			}


			if (!is_down_stair)
			{
				//LOG_DEBUG("CharacterController::move: horizontal pass no hit, moving normally");
				final_position += horizontal_displacement;
			}
		}

		LOG_DEBUG("CharacterController::move: m_is_touch_ground = {} {}", m_is_touch_ground, physics_scene->isOverlap(m_rigidbody_shape, world_transform.getMatrix()));
		m_is_touch_ground = temp_m_is_touch_ground;
		m_is_falling = !m_is_touch_ground;
		//if (m_is_touch_ground) {
		//	m_is_falling = false;
		//}
		//if (!m_is_touch_ground && !is_down_stair) {
		//	LOG_DEBUG("CharacterController::move: m_is_falling");
		//	m_is_falling = true;
		//}
		//if (!physics_scene->sweep(
		//	m_rigidbody_shape,
		//	world_transform.getMatrix(),
		//	Vector3::NEGATIVE_UNIT_Z,
		//	0.005f,
		//	hits))
		//{
		//	m_is_falling = true;
		//	LOG_DEBUG("CharacterController::move: m_is_falling");
		//}
		//else
		//{
		//	m_is_falling = false;
		//	LOG_DEBUG("CharacterController::move: m_is_touch_ground {}",hits[0].hit_distance);
		//}

		//// 0代表正常，1代表正常平移移动卡住，2代表上楼梯卡住，3代表平移和爬楼都不行，面前有大障碍物
		//enum CantMoveType
		//{
		//	Normal = 0,
		//	CantMove = 1,
		//	CantStepAndCantMove = 3
		//};
		//CantMoveType cantMove = Normal;
		//if (physics_scene->sweep(
		//	m_rigidbody_shape,
		//	world_transform.getMatrix(),
		//	horizontal_direction,
		//	horizontal_displacement.length(),
		//	hits))
		//{
		//	final_position += hits[0].hit_distance * horizontal_direction;
		//	if (hits[0].hit_distance <= 0.01) {
		//		cantMove = CantMove;
		//		//LOG_INFO("cant move+1");
		//	}
		//}
		//else
		//{
		//	final_position += horizontal_displacement;
		//}

		////首先判断是不是上楼梯？前面是不是有可以走上去的障碍物
		//float max_step_height = 1.0f; // 最大可爬行高度

		////先移动位置到可爬行最大高度
		//Transform step_transform = Transform(
		//	current_position + horizontal_displacement + max_step_height * Vector3::UNIT_Z,
		//	Quaternion::IDENTITY,
		//	Vector3::UNIT_SCALE);

		////先判断最大步高的情况，如果还是有障碍物重合，则不能上楼梯，其实不是很严谨
		//if (physics_scene->isOverlap(m_rigidbody_shape, step_transform.getMatrix()))
		//{
		//	cantMove = CantStepAndCantMove;
		//}
		//if (cantMove == CantMove)
		//{
		//	// 找到碰到地面的高度
		//	for (float i = 0.0f; i <= 2.0f; i += 0.01f)
		//	{
		//		if (physics_scene->sweep(
		//			m_rigidbody_shape,
		//			step_transform.getMatrix(),
		//			Vector3::NEGATIVE_UNIT_Z * i,
		//			abs(i),
		//			hits))
		//		{
		//			final_position += horizontal_displacement + Vector3::UNIT_Z * (max_step_height - i);
		//			break;
		//		}
		//	}
		//}



		//// 沿着墙壁滑动
		//// 思路：当无法运动也不是上楼梯时，找可以移动的方向移动，即运动的方向没有障碍物
		//// 又移动的方向应该和输入方向夹角较小，因此扫描移动方向-90~+90范围,从0开始向正负方向扫描，找到较小的偏转角度方向移动
		//if (cantMove == CantStepAndCantMove)
		//{
		//	//LOG_INFO("cant move");
		//	float originx = horizontal_direction.x;
		//	float originy = horizontal_direction.y;
		//	for (float delta = 0; delta <= Math_HALF_PI + Math_fDeg2Rad; delta += Math_fDeg2Rad)
		//	{
		//		float x = cos(delta) * originx - sin(delta) * originy;
		//		float y = cos(delta) * originy + sin(delta) * originx;
		//		Vector3 revise_horizontal_direction = Vector3(x, y, 0).normalisedCopy();
		//		if (!physics_scene->sweep(
		//			m_rigidbody_shape,
		//			world_transform.getMatrix(),
		//			revise_horizontal_direction,
		//			horizontal_displacement.length(),
		//			hits))
		//		{
		//			final_position += revise_horizontal_direction * horizontal_displacement.length();
		//			break;
		//		}

		//		x = cos(delta) * originx + sin(delta) * originy;
		//		y = cos(delta) * originy - sin(delta) * originx;
		//		revise_horizontal_direction = Vector3(x, y, 0).normalisedCopy();
		//		if (!physics_scene->sweep(
		//			m_rigidbody_shape,
		//			world_transform.getMatrix(),
		//			revise_horizontal_direction,
		//			horizontal_displacement.length(),
		//			hits))
		//		{
		//			final_position += revise_horizontal_direction * horizontal_displacement.length();
		//			break;
		//		}
		//	}
		//}


		int flag = 0;
		//当不在地面上并且卡住时，使用水平方向，前后探测到一个不卡的位置设置为最终位置
		if (!temp_m_is_touch_ground && physics_scene->isOverlap(m_rigidbody_shape, world_transform.getMatrix())) {
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
		//LOG_INFO(std::to_string(cantMove));
		return final_position;
	}

} // namespace Pilot
