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
      q := qs[jnt.Src.Name][0]      // [q, dq, ddq]
      lnk.State.ApplyJoint(jnt.Type, q)
      jnt.UpdateR(q) 
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
  //jac.Print()
  //MatPrint(jac) 
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

/*
func (v *Link) rnea(w, al, ac, ae *mat.Dense, g float64) (f, tau *mat.Dense) {
  // current velocities 
  
  
  // current torques/forces  
}*/ 

type Joint struct {
  // source 
  Src          *urdf.Joint 
  // tree 
  Parent       *Link 
  Child        *Link 
  // type 
  Type          JointType
  // Transformations 
  Trans         Transform     // constant transformation
  Limit         [2]float64
  Rij           *mat.Dense    // from current to next joint 
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
  jnt.Rij = eye3()
  jnt.Rij.Copy(jnt.Trans.Rot) 
  return jnt
}

// Update current rotation transform 
func (jnt *Joint) UpdateR(q float64) {
  switch jnt.Type {
  case joint_Rx:
    jnt.Rij.Mul(jnt.Trans.Rot, Rx(q))
  case joint_Ry:
    jnt.Rij.Mul(jnt.Trans.Rot, Ry(q))
  case joint_Rz:
    jnt.Rij.Mul(jnt.Trans.Rot, Rz(q))
  }
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

