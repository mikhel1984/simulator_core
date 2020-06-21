package rigid 

import (
  //"gonum.org/v1/gonum/mat"
  "../mat"   // temporary
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
  //Rot  *mat.Dense  // rotation
  //Pos  *mat.Dense  // translation 
  Rot  *mat.Matrix
  Pos  *mat.Matrix
}

func (t *Transform) Reset() {
  //t.Rot = mat.NewDense(3,3, []float64{
  t.Rot = mat.New(3,3, []float64{
    1, 0, 0,
    0, 1, 0,
    0, 0, 1})
  //t.Pos = mat.NewDense(3,1, []float64{0,0,0})
  t.Pos = mat.New(3,1, []float64{0,0,0})
}

func (t *Transform) Set(src *Transform) {
  t.Rot = src.Rot.Copy()
  t.Pos = src.Pos.Copy() 
}

func (dst *Transform) Apply(t *Transform) {
  dst.Pos = mat.Sum(mat.Prod(dst.Rot,t.Pos), dst.Pos)
  dst.Rot = mat.Prod(dst.Rot,t.Rot)
}


func GetTransform(jt JointType, q float64) *Transform {
  res := Transform {} 
  res.Reset() 
  switch jt {
  case joint_Tx:
    res.Pos = Txyz(q,0,0) 
  case joint_Ty:
    res.Pos = Txyz(0,q,0)
  case joint_Tz:
    res.Pos = Txyz(0,0,q)
  case joint_Rx:
    res.Rot = Rx(q)
  case joint_Ry:
    res.Rot = Ry(q)
  case joint_Rz:
    res.Rot = Rz(q) 
  //default:
  }
  return &res 
}

func (dst *Transform) ApplyJoint(tp JointType, q float64) {
  switch tp {
  case joint_Tx:
    dst.Pos = mat.Sum(mat.Prod(dst.Rot,Txyz(q,0,0)),dst.Pos) 
  case joint_Ty:
    dst.Pos = mat.Sum(mat.Prod(dst.Rot,Txyz(0,q,0)),dst.Pos) 
  case joint_Tz:
    dst.Pos = mat.Sum(mat.Prod(dst.Rot,Txyz(0,0,q)),dst.Pos) 
  case joint_Rx:
    dst.Rot = mat.Prod(dst.Rot,Rx(q)) 
  case joint_Ry:
    dst.Rot = mat.Prod(dst.Rot,Ry(q))
  case joint_Rz:
    dst.Rot = mat.Prod(dst.Rot,Rz(q))
  }
}

func (t *Transform) toColumn(m *mat.Matrix, col int, tp JointType, ee *mat.Matrix) {
  switch tp {
  case joint_Tx, joint_Ty, joint_Tz:
    z := t.Rot.Col(int(tp))
    m.Block(0,col, 3,1).Insert(z) 
  case joint_Rx, joint_Ry, joint_Rz:
    z := t.Rot.Col(int(tp)-3).Copy()
    m.Block(3,col, 3,1).Insert(z)
    w := z.Cross(mat.Diff(ee, t.Pos))
    m.Block(0,col, 3,1).Insert(w)
  }
}

/*
func (t *Transform) ToH() *mat.Dense {
  return mat.NewDense(4,4, []float64{
    t.Rot.At(0,0), t.Rot.At(0,1), t.Rot.At(0,2), t.Pos.At(0,0),
    t.Rot.At(1,0), t.Rot.At(1,1), t.Rot.At(1,2), t.Pos.At(1,0),
    t.Rot.At(2,0), t.Rot.At(2,1), t.Rot.At(2,2), t.Pos.At(2,0),
                0,             0,             0,            1})
}*/

//func Rx(q float64) *mat.Dense {
func Rx(q float64) *mat.Matrix {
  s,c := math.Sin(q), math.Cos(q)
  //return mat.NewDense(3,3, []float64{
  return mat.New(3,3, []float64{
     1, 0, 0,
     0, c,-s,
     0, s, c})
}

//func Ry(q float64) *mat.Dense {
func Ry(q float64) *mat.Matrix {
  s,c := math.Sin(q), math.Cos(q)
  //return mat.NewDense(3,3, []float64{
  return mat.New(3,3, []float64{
     c, 0, s,
     0, 1, 0,
    -s, 0, c})
}

//func Rz(q float64) *mat.Dense {
func Rz(q float64) *mat.Matrix {
  s,c := math.Sin(q), math.Cos(q)
  //return mat.NewDense(3,3, []float64{
  return mat.New(3,3, []float64{
     c,-s, 0,
     s, c, 0,
     0, 0, 1})
}

//func RPY(w,p,r float64) *mat.Dense {
func RPY(w,p,r float64) *mat.Matrix {
  // yaw - X, pitch - Y, roll - Z
  sw, cw := math.Sin(w), math.Cos(w)
  sp, cp := math.Sin(p), math.Cos(p)
  sr, cr := math.Sin(r), math.Cos(r)
  ssw, scw := sp*sw, sp*cw
  //return mat.NewDense(3,3, []float64{
  return mat.New(3,3, []float64{
    cr*cp,-sr*cw+cr*ssw, sr*sw+cr*scw, 
    sr*cp, cr*cw+sr*ssw,-cr*sw+sr*scw,
      -sp, cp*sw,        cp*cw })
}

//func Txyz(x,y,z float64) *mat.Dense {
//  return mat.NewDense(3,1, []float64{x,y,z})
//}
  
func Txyz(x,y,z float64) *mat.Matrix {
  return mat.New(3,1, []float64{x,y,z})
}

func jacEmpty(cols int) *mat.Matrix {
  return mat.New(6,cols,nil)
}

