package rigid 

import (
  "../urdf"  
)

type Link struct {
  // source 
  Src          *urdf.Link 
  // tree 
  Joints       []*Joint 
  Parent       *Joint 
  // current state
  State         Transform
} 

func linkFromModel(m *urdf.Link) *Link {
  lnk := new(Link) 
  lnk.Src = m
  lnk.State.Reset()
  return lnk
}

func (v *Link) UpdateState(qs map[string][3]float64) {
  // update next links
  for _,jnt := range v.Joints {
    lnk := jnt.Child 
    lnk.State.Set(&v.State)         // copy previous state
    lnk.State.Apply(&jnt.Trans)     // displace
    if jnt.Type != joint_Fixed {    // apply joint transformation
      q := qs[jnt.Src.Name][0]      // [q, dq, ddq]
      lnk.State.ApplyJoint(jnt.Type, q)
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

func (v *Link) Jacobian(mov []*Joint) {
  if mov == nil {
    mov = v.Predecessors()
  }
  jac := jacEmpty(len(mov)) 
  for i,jnt := range mov {
    jnt.Child.State.toColumn(jac, i, jnt.Type, v.State.Pos)
  }
  //return jac
  //jac.Print()
  MatPrint(jac) 
}

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

type Joint struct {
  // source 
  Src          *urdf.Joint 
  // tree 
  Parent       *Link 
  Child        *Link 
  // type 
  Type          JointType
  // current state
  Angle         float64 
  Velocity      float64
  Acceleration  float64 
  // constant transformation 
  Trans         Transform 
  Limit         [2]float64
}

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
  //jnt.Trans.Rot.Print()
  //jnt.Trans.Pos.Print()
  return jnt
}

func MakeTree(model *urdf.Model) *Link {
  // read links 
  links := make(map[string]*Link)   
  for i := 0; i < len(model.Links); i++ {
    //lnk := new(Link)  
    //lnk.Src = &model.Links[i]
    //lnk.fromModel(&model.Links[i])
    lnk := linkFromModel(&model.Links[i])
    links[ model.Links[i].Name ] = lnk     
  }
  // joints   
  for i := 0; i < len(model.Joints); i++ {
    //jnt := new(Joint)
    //jnt.Src = &model.Joints[i]
    jnt := jointFromModel(&model.Joints[i])
    lnk := links[model.Joints[i].Parent.Name] 
    jnt.Parent = lnk     
    lnk.Joints = append(lnk.Joints, jnt) 
    
    lnk = links[model.Joints[i].Child.Name]
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

