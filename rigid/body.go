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
  M     float64   // link mass 
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
      jnt.UpdateR(qlst[0])                // rotation between current and next links 
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
func (v *Link) Jacobian(mov []*Joint) *mat.Dense {
  if mov == nil {
    mov = v.Predecessors()
  }
  jac := jacEmpty(len(mov)) 
  for i,jnt := range mov {
    jnt.Child.State.toColumn(jac, i, jnt.Type, v.State.Pos)
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

func (v *Link) rnea(w, dw, ae *mat.Dense, g float64) (*mat.Dense,*mat.Dense) {
  //if len(v.Joints) == 0 {
  //  return zero31(), zero31()  // last link
  //}
  var wi, dwi, aci, aei *mat.Dense
  // parent
  jnt := v.Parent   
  if jnt != nil {
    wi, dwi = jnt.getAngularAcc(w,dw)
    aci = jnt.getCenterAcc(ae, w, dw, v.Dyn.Rc) 

  } else {
    wi, dwi, aci = zero31(), zero31(), zero31()    
  } 
  // force / torque
  var fi, taui, tmp mat.Dense
  fi.Add(wi,Cross(aci,v.Dyn.Rc))
  fi.Add(&fi,Cross(wi,Cross(wi,v.Dyn.Rc)))
  taui.Mul(v.Dyn.I,dwi)
  tmp.Mul(v.Dyn.I,wi)
  taui.Add(&taui,Cross(wi,&tmp))
  taui.Add(&taui,Cross(v.Dyn.Rc,&fi))
  // children 
  for _,jc := range v.Joints {
    re := jc.Trans.Pos
    aei = jnt.getEndAcc(ae,w,dw, re)
    f, tau := jc.Child.rnea(wi,dwi,aei,g)
    // force
    var fc mat.Dense
    fc.Mul(jc.Local.Rot, f) 
    fi.Add(&fi,&fc)
    // torque
    tmp.Mul(jc.Local.Rot, tau)
    taui.Add(&taui,&tmp) 
    tmp.Sub(jnt.Trans.Pos,v.Dyn.Rc)
    taui.Add(&taui,Cross(&tmp,&fc))    
  }

  if jnt.Type != joint_Fixed {
    jnt.Tau = 0
    for i := 0; i < 3; i++ {
      jnt.Tau += taui.At(i,0) * jnt.Local.Pos.At(i,0)
    }
  // add friction
  }
  
  return &fi, &taui  
}

func (v *Link) DynUpdate(g float64) {
  zer := zero31()
  v.rnea(zer, zer, zer, g)
}

func ReadTorquesFor(mov []*Joint) *mat.Dense {
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
  Local         Transform     // rotation from current to next and joint axis 
  Limit         [2]float64
  //Rij           *mat.Dense    // from current to next joint 
  //Axis          *mat.Dense    // joint axis 
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
  jnt.Local.Pos = getJointAxis(jnt.Type)
  return jnt
}

// Update current rotation transform 
func (jnt *Joint) UpdateR(q float64) {
  switch jnt.Type {
  case joint_Rx:
    jnt.Local.Rot.Mul(jnt.Trans.Rot, Rx(q))
  case joint_Ry:
    jnt.Local.Rot.Mul(jnt.Trans.Rot, Ry(q))
  case joint_Rz:
    jnt.Local.Rot.Mul(jnt.Trans.Rot, Rz(q))
  }
}

func (jnt *Joint) getAngularAcc(wp, dwp *mat.Dense) (*mat.Dense,*mat.Dense) {
  var wi, dwi, zqd, zq2d mat.Dense 
  zqd.Scale(jnt.Vel, jnt.Local.Pos)
  zq2d.Scale(jnt.Acc, jnt.Local.Pos) 
  Rt := jnt.Local.Rot.T()
  wi.Mul(Rt, wp)
  dwi.Mul(Rt, dwp) 
  switch jnt.Type {
  case joint_Rx, joint_Ry, joint_Rz:          
    dwi.Add(&dwi, &zq2d)
    dwi.Add(&dwi, Cross(&wi, &zqd))  
    wi.Add(&wi, &zqd)     
  }
  return &wi, &dwi 
}

func (jnt *Joint) getCenterAcc(ap, wi, dwi, rc *mat.Dense) *mat.Dense {
  var ai mat.Dense 
  //Rt := jnt.Local.Rot.T()
  ai.Mul(jnt.Local.Rot.T(), ap)       
  ai.Add(&ai, Cross(dwi,rc))
  ai.Add(&ai, Cross(wi,Cross(wi,rc)))
  return &ai  
}

func (jnt *Joint) getEndAcc(ap, wi, dwi, r *mat.Dense) *mat.Dense {
  var ai mat.Dense 
  //Rt := jnt.Local.Rot.T()
  // TODO: write for prismatic joint
  ai.Mul(jnt.Local.Rot.T(), ap)  
  ai.Add(&ai, Cross(dwi,r))
  ai.Add(&ai, Cross(wi,Cross(wi,r)))
  return &ai 
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

