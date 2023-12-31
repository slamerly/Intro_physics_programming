#pragma once
#include "code/Math/Matrix.h"
#include "code/Math/Bounds.h"
#include "code/Math/Quat.h"

class Shape {
public:
	enum class ShapeType
	{
		SHAPE_SPHERE,
	};

	virtual ShapeType GetType() const = 0;
	virtual Mat3 InertiaTensor() const = 0;
	virtual Vec3 GetCenterOfMass() const { return massCenter; }
	virtual Bounds GetBounds(const Vec3& pos, const Quat& orient) const = 0;
	virtual Bounds GetBounds() const = 0;

protected:
	Vec3 massCenter;
};

class ShapeSphere : public Shape {
public:
	ShapeSphere(float radiusP) : radius(radiusP)
	{
		massCenter.Zero();
	}
	
	Mat3 InertiaTensor() const override;
	ShapeType GetType() const override { return ShapeType::SHAPE_SPHERE; }
	
	Bounds GetBounds(const Vec3& pos, const Quat& orient) const override;
	Bounds GetBounds() const override;

	float radius;
};

