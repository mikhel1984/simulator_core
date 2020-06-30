package rigid 

import (
  "gonum.org/v1/gonum/mat"  
  "math"
  "fmt"
)

// Joint classification 
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

// Replace homogenous matrix with pair (R,p) 
type Transform struct {
  Rot  *mat.Dense  // rotation
  Pos  *mat.Dense  // translation   
}

// Get identity matrix 3x3 
func eye33() *mat.Dense {
  return mat.NewDense(3,3, []float64{
    1, 0, 0,
    0, 1, 0,
    0, 0, 1})
}

func zero31() *mat.Dense {
  return mat.NewDense(3,1, []float64{0,0,0}) 
}

// Set R = I and p = 0
func (t *Transform) Reset() {
  t.Rot = eye33() 
  t.Pos = zero31()  
}


// Copy Transform object 
func (t *Transform) Set(src *Transform) {
  matInsert(0,0, t.Rot, src.Rot)
  matInsert(0,0, t.Pos, src.Pos)
}

// Update state using the given transformation 
func (dst *Transform) Apply(t *Transform) {
  var tmp mat.Dense 
  tmp.Mul(dst.Rot, t.Pos) 
  dst.Pos.Add(&tmp, dst.Pos)    // p2 += R2*p1 
  dst.Rot.Mul(dst.Rot, t.Rot)   // R2 *= R1   
}

/*
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
*/

// Apply joint transformation 
func (dst *Transform) ApplyJoint(tp JointType, q float64) {
  var tmp mat.Dense
  switch tp {
  case joint_Tx:
    tmp.Mul(dst.Rot, Txyz(q,0,0))
    dst.Pos.Add(&tmp,dst.Pos)    
  case joint_Ty:
    tmp.Mul(dst.Rot,Txyz(0,q,0))
    dst.Pos.Add(&tmp,dst.Pos)    
  case joint_Tz:
    tmp.Mul(dst.Rot, Txyz(0,0,q))
    dst.Pos.Add(&tmp,dst.Pos)    
  case joint_Rx:
    dst.Rot.Mul(dst.Rot, Rx(q))    
  case joint_Ry:    
    dst.Rot.Mul(dst.Rot, Ry(q))
  case joint_Rz:    
    dst.Rot.Mul(dst.Rot, Rz(q))
  }
}

// Place one matrix into another 
func matInsert(r,c int, dst *mat.Dense, src mat.Matrix) {
  nr, nc := src.Dims()
  for i := 0; i < nr; i++ {
    for j := 0; j < nc; j++ {
      dst.Set(r+i,c+j,src.At(i,j))
    }
  }
}

func Cross(a,b mat.Matrix) *mat.Dense {
  return mat.NewDense(3,1, []float64 {
    a.At(1,0)*b.At(2,0) - a.At(2,0)*b.At(1,0),
  -(a.At(0,0)*b.At(2,0) - a.At(2,0)*b.At(0,0)),
    a.At(0,0)*b.At(1,0) - a.At(1,0)*b.At(0,0)})
}

func (t *Transform) toColumn(m *mat.Dense, col int, tp JointType, ee *mat.Dense) {  
  switch tp {
  case joint_Tx, joint_Ty, joint_Tz:    
    z := t.Rot.Slice(0,3, int(tp), int(tp)+1)
    matInsert(0,col, m, z)
  case joint_Rx, joint_Ry, joint_Rz:
    z := t.Rot.Slice(0,3, int(tp)-3, int(tp)-2)
    matInsert(3,col, m, z)
    var tmp mat.Dense 
    tmp.Sub(ee, t.Pos)
    w := Cross(z, &tmp)
    matInsert(0,col, m, w)
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
  
//func Txyz(x,y,z float64) *mat.Matrix {
//  return mat.New(3,1, []float64{x,y,z})
//}

func jacEmpty(cols int) *mat.Dense {
  return mat.NewDense(6,cols,nil)
}

func MatPrint(m *mat.Dense) {
  f := mat.Formatted(m, mat.Prefix(" "), mat.Squeeze())
  fmt.Printf("%v", f) 
}

 