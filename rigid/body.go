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
  // current state
  jnt := v.Parent
  if jnt != nil {
    v.State.Set(&jnt.Parent.State) 
    if jnt.Type != joint_Fixed {
      q := qs[jnt.Src.Name][0] 
      v.State.Apply(GetTransform(jnt.Type,q))
    }
    v.State.Apply(&jnt.Trans)
  }
  // next elements 
  for _, j := range v.Joints {
    j.Child.UpdateState(qs)
  }
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

