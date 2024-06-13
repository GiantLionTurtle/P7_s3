
#ifndef P7_BOUNDINGBOX_HPP_
#define P7_BOUNDINGBOX_HPP_

// 2d position, by convention starts on the floor 
// at home position (on tree side)
struct Position {
  double x { 0.0 }, y { 0.0 };

  Position() = default;
  Position(double x_, double y_)
    : x(x_)
    , y(y_)
  {

  }
};

// 2d bounding box
struct BoundingBox {
  Position topLeft, bottomRight;

  BoundingBox() = default;
  BoundingBox(Position topLeft_, Position bottomRight_)
    : topLeft(topLeft_)
    , bottomRight(bottomRight_)
  {

  }

  bool contains(Position pos) const
  {
    return  pos.x > topLeft.x && pos.x < bottomRight.x &&
            pos.y < topLeft.y && pos.y > bottomRight.y;
  }
};

#endif