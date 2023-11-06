#pragma once
class Shape {
public:
	enum class ShapeType
	{
		SHAPE_SPHERE,
	};

	virtual ShapeType GetType() const = 0;
	virtual Mat3 InertiaTensor() const = 0;
	Vec3 GetCenterOfMass() const { return massCenter; }

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
	
	float radius;
};

