package rigid 

import (
  "../urdf" 
  "gonum.org/v1/gonum/mat"  
)

type Link struct {
  // source 
  Src          *urdf.Link 
  // tree 
  Joints       []*Joint 
  Parent       *Joint 
  // current state
  State         Transform
  // inertial parameters 
  Dyn           Inertial 
} 

// Inertial parameters of link 
type Inertial struct {
  I     *mat.Dense   // inertia matrix 
  Rc    *mat.Dense   // mass center 
  M     float64      // link mass 
}

// Link constructor based on URDF
func linkFromModel(m *urdf.Link) *Link {
  lnk := new(Link) 
  lnk.Src = m
  lnk.State.Reset()
  
  // inertial parameters 
  lnk.Dyn.M = m.GetMass()  
  lnk.Dyn.Rc = mat.NewDense(3,1, m.GetMassCenter()) 
  ii := m.GetInertia() 
  lnk.Dyn.I = mat.NewDense(3,3, []float64 {
      ii[0],ii[1],ii[2],
      ii[1],ii[3],ii[4],
      ii[2],ii[4],ii[5]})
      
  return lnk
}

// Update chain parameters for the given joint states 
func (v *Link) UpdateState(qs map[string][3]float64) {
  // update next links
  for _,jnt := range v.Joints {
    lnk := jnt.Child 
    lnk.State.Set(&v.State)         // copy previous state
    lnk.State.Apply(&jnt.Trans)     // displace
    if jnt.Type != joint_Fixed {    // apply joint transformation
      qlst := qs[ jnt.Src.Name ]    // [q, dq, ddq]
      jnt.Angle, jnt.Vel, jnt.Acc = qlst[0], qlst[1], qlst[2]      
      lnk.State.ApplyJoint(jnt.Type, qlst[0])  // new joint origin 
      jnt.UpdateLocal(qlst[0])                // rotation between current and next links 
    }
    // next elements
    lnk.UpdateState(qs)
  }
}

// collect movable joints
func (v *Link) Predecessors() []*Joint {
  var acc []*Joint 
  jnt := v.Parent
  for jnt != nil {
    if jnt.Type != joint_Fixed {
      acc = append(acc, jnt)
    }
    jnt = jnt.Parent.Parent
  }
  // reverse
  n := len(acc)
  res := make([]*Joint,n)
  for i := 0; i < n; i++ {
    res[i] = acc[n-i-1] 
  }
  return res
}

// Calculate Jacobian matrix 
func (ee *Link) Jacobian(mov []*Joint) *mat.Dense {
  if mov == nil {
    mov = ee.Predecessors()
  }
  jac := jacEmpty(len(mov)) 
  for i,jnt := range mov {
    jnt.Child.State.toColumn(jac, i, jnt.Type, ee.State.Pos)
  }
  return jac
}

// Find link with the given name 
func (v *Link) Find(name string) *Link {
  if v.Src.Name == name {
    return v
  }
  for _,j := range v.Joints {
    res := j.Child.Find(name) 
    if res != nil {
      return res
    }
  }
  return nil
}

// Use recursive Newton-Euler dymanic calculation
func (v *Link) rnea(w, dw, ae *mat.Dense) (*mat.Dense,*mat.Dense) {
  jnt := v.Parent 
  wi, dwi := jnt.getAngularAcc(w,dw) 
  aci := jnt.getLinearAcc(ae, wi, dwi, v.Dyn.Rc) 
  // force / torque
  var fi, taui, tmp, fc mat.Dense
  fi.Scale(v.Dyn.M,aci)
  taui.Mul(v.Dyn.I,dwi)
  tmp.Mul(v.Dyn.I,wi);   taui.Add(&taui,Cross(wi,&tmp))
  // children 
  for _,jc := range v.Joints {
    aei := jnt.getLinearAcc(ae,wi,dwi, jc.Local.Pos) 
    f, tau := jc.Child.rnea(wi,dwi,aei)
    // force    
    fc.Mul(jc.Local.Rot,f)
    fi.Add(&fi,&fc)
    // torque
    tmp.Mul(jc.Local.Rot, tau);     taui.Add(&taui,&tmp)
    tmp.Sub(v.Dyn.Rc,jc.Local.Pos); taui.Add(&taui,Cross(&fc,&tmp))
  }
  taui.Sub(&taui,Cross(&fi,v.Dyn.Rc))

  if jnt != nil {
    switch jnt.Type {
    case joint_Rx, joint_Ry, joint_Rz:
      jnt.Tau = taui.At(int(jnt.Type)-3,0)
    case joint_Tx, joint_Ty, joint_Tz:
      jnt.Tau = fi.At(int(jnt.Type),0) 
    }
    // add friction
  }  
    
  return &fi, &taui  
}

// Find dymanical state using RNEA algorithm
func (base *Link) UpdateDyn(g float64) {
  zer := zero31()
  acc := zero31()
  acc.Set(2,0,g)  
  base.rnea(zer, zer, acc)
}

func ReadTorques(mov []*Joint) *mat.Dense {
  res := mat.NewDense(len(mov),1,nil)
  for i,v := range mov {
    res.Set(i,0, v.Tau)
  }
  return res
}

type Joint struct {
  // source 
  Src          *urdf.Joint 
  // tree 
  Parent       *Link 
  Child        *Link 
  // type 
  Type          JointType
  // Parameters 
  Angle         float64
  Vel           float64
  Acc           float64
  // Transformations 
  Trans         Transform     // constant transformation
  Local         Transform     // rotation from current to next 
  Limit         [2]float64
  //Rij           *mat.Dense    // from current to next joint 
  Axis          *mat.Dense    // joint axis 
  // dynamics 
  Tau           float64 
}

// Joint constructor from URDF 
func jointFromModel(m *urdf.Joint) *Joint {
  jnt := new(Joint) 
  jnt.Src = m    
  if m.Type == "revolute" {
    a := m.GetAxis() 
    jnt.Type = JointType(3+a)     
    jnt.Limit[0], jnt.Limit[1] = m.GetLimits()
  } else if m.Type == "prismatic" {
    a := m.GetAxis()
    jnt.Type = JointType(a) 
    jnt.Limit[0], jnt.Limit[1] = m.GetLimits()
  } else {
    jnt.Type = joint_Fixed;
  } 
  // transformation to next joint
  v := m.GetXyz() 
  jnt.Trans.Pos = Txyz(v[0],v[1],v[2]) 
  v = m.GetRpy()
  jnt.Trans.Rot = RPY(v[0],v[1],v[2])
  // local transformation 
  jnt.Local.Rot = eye33()  
  jnt.Local.Rot.Copy(jnt.Trans.Rot) 
  jnt.Local.Pos = zero31()
  jnt.Local.Pos.Copy(jnt.Trans.Pos)
  jnt.Axis = getJointAxis(jnt.Type)
  return jnt
}

// Update current rotation transform 
func (jnt *Joint) UpdateLocal(q float64) {
  switch jnt.Type {
  case joint_Rx:
    jnt.Local.Rot.Mul(jnt.Trans.Rot, Rx(q))
  case joint_Ry:
    jnt.Local.Rot.Mul(jnt.Trans.Rot, Ry(q))
  case joint_Rz:
    jnt.Local.Rot.Mul(jnt.Trans.Rot, Rz(q))
  case joint_Tx:
    jnt.Local.Pos.Add(jnt.Trans.Pos, Txyz(q,0,0))
  case joint_Ty:
    jnt.Local.Pos.Add(jnt.Trans.Pos, Txyz(0,q,0))
  case joint_Tz:
    jnt.Local.Pos.Add(jnt.Trans.Pos, Txyz(0,0,q))
  }
}

func (jnt *Joint) getAngularAcc(wp, dwp *mat.Dense) (*mat.Dense,*mat.Dense) {
  if jnt == nil {
    return wp, dwp 
  }
  var wi, dwi, zqd, zq2d mat.Dense 
  zqd.Scale(jnt.Vel, jnt.Axis)
  zq2d.Scale(jnt.Acc, jnt.Axis)
  Rt := jnt.Local.Rot.T()  
  wi.Mul(Rt, wp)
  dwi.Mul(Rt, dwp) 
  switch jnt.Type {
  case joint_Rx, joint_Ry, joint_Rz:
    wi.Add(&wi, &zqd) 
    dwi.Add(&dwi, &zq2d)
    dwi.Add(&dwi, Cross(&wi, &zqd))
  }
  return &wi, &dwi
}

func (jnt *Joint) getLinearAcc(ap, wi, dwi, r *mat.Dense) *mat.Dense {
   var ai mat.Dense   
  // TODO: write for prismatic joint
  if jnt != nil {
    ai.Mul(jnt.Local.Rot.T(), ap)
  } else {
    ai.Scale(1,ap)
  }
  ai.Add(&ai, Cross(dwi,r))
  ai.Add(&ai, Cross(wi,Cross(wi,r)))
  return &ai 
}

func (jnt *Joint) InRange(q float64) bool {
  return jnt.Limit[0] <= q && q <= jnt.Limit[1] 
}


func getJointAxis(tp JointType) *mat.Dense {
  switch tp {
  case joint_Tx, joint_Rx:
    return mat.NewDense(3,1, []float64{1,0,0})
  case joint_Ty, joint_Ry:
    return mat.NewDense(3,1, []float64{0,1,0})
  case joint_Tz, joint_Rz:
    return mat.NewDense(3,1, []float64{0,0,1})  
  }
  return mat.NewDense(3,1, []float64{0,0,0}) 
}

// Make tree of rigid body elements 
func BodyTree(model *urdf.Model) *Link {
  // read links 
  links := make(map[string]*Link)   
  for i := 0; i < len(model.Links); i++ {
    lnk := linkFromModel(&model.Links[i])
    links[ model.Links[i].Name ] = lnk     
  }
  // joints   
  for i := 0; i < len(model.Joints); i++ {
    jnt := jointFromModel(&model.Joints[i])
    lnk := links[ model.Joints[i].Parent.Name ] 
    jnt.Parent = lnk     
    lnk.Joints = append(lnk.Joints, jnt) 
    
    lnk = links[ model.Joints[i].Child.Name ]
    lnk.Parent = jnt
    jnt.Child = lnk     
  }
  // find "free" link 
  for _,v := range links {
    if v.Parent == nil {
      return v       
    }
  }
  
  return nil
} 

