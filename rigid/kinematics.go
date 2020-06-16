package rigid 

import (
  "gonum.org/v1/gonum/mat"
  "math"
)

type JointType int
const (
  joint_Tx JointType = iota 
  joint_Ty
  joint_Tz
  joint_Rx
  joint_Ry
  joint_Rz
  joint_Fixed
)

type Transform struct {
  Rot  *mat.Dense  // rotation
  Pos  *mat.Dense  // translation 
}

func (t *Transform) Reset() {
  t.Rot = mat.NewDense(3,3, []float64{
    1, 0, 0,
    0, 1, 0,
    0, 0, 1})
  t.Pos = mat.NewDense(3,1, []float64{0,0,0})
}

func (t *Transform) ToH() *mat.Dense {
  return mat.NewDense(4,4, []float64{
    t.Rot.At(0,0), t.Rot.At(0,1), t.Rot.At(0,2), t.Pos.At(0,0),
    t.Rot.At(1,0), t.Rot.At(1,1), t.Rot.At(1,2), t.Pos.At(1,0),
    t.Rot.At(2,0), t.Rot.At(2,1), t.Rot.At(2,2), t.Pos.At(2,0),
                0,             0,             0,            1})
}

func Rx(q float64) *mat.Dense {
  s,c := math.Sin(q), math.Cos(q)
  return mat.NewDense(3,3, []float64{
     1, 0, 0,
     0, c,-s,
     0, s, c})
}

func Ry(q float64) *mat.Dense {
  s,c := math.Sin(q), math.Cos(q)
  return mat.NewDense(3,3, []float64{
     c, 0, s,
     0, 1, 0,
    -s, 0, c})
}

func Rz(q float64) *mat.Dense {
  s,c := math.Sin(q), math.Cos(q)
  return mat.NewDense(3,3, []float64{
     c,-s, 0,
     s, c, 0,
     0, 0, 1})
}

func RPY(w,p,r float64) *mat.Dense {
  // yaw - X, pitch - Y, roll - Z
  sw, cw := math.Sin(w), math.Cos(w)
  sp, cp := math.Sin(p), math.Cos(p)
  sr, cr := math.Sin(r), math.Cos(r)
  ssw, scw := sp*sw, sp*cw
  return mat.NewDense(3,3, []float64{
    cr*cp,-sr*cw+cr*ssw, sr*sw+cr*scw, 
    sr*cp, cr*cw+sr*ssw,-cr*sw+sr*scw,
      -sp, cp*sw,        cp*cw })
}

func Txyz(x,y,z float64) *mat.Dense {
  return mat.NewDense(3,1, []float64{x,y,z})
}
  
