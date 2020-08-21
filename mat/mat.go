// Matrix main definitions
package mat

import (
   "math"
   "fmt"
)

// Common representation
type Matrix struct {
   // matrix size
   rows, cols int
   // content
   data       []float64 
}

/**************** Interface methods ********************/ 

// Some specific methods 
type IMatrix interface {
   at(r,c int) *float64        // element access
   wrongIndex(r,c int) bool    // index checking
   Dim() (r,c int)             // matrix dimention
   Copy() *Matrix              // save as "normal" matrix 
}

// Get size
func (m Matrix) Dim() (r,c int) {
   return m.rows, m.cols
}

// Access to element 
func (m *Matrix) at(r,c int) *float64 {
   return &m.data[ r*m.cols + c ];
}

// Index checking 
func (m Matrix) wrongIndex(r,c int) bool {
   return r < 0 || r >= m.rows || c < 0 || c >= m.cols
}

// Create equal matrix
func (m Matrix) Copy() *Matrix {
   res := emptyMatrix(m.rows, m.cols)
   copy(res.data, m.data)
   return res
}

func (dst *Matrix) CopyFrom(src *Matrix) {
  if len(src.data) != len(dst.data) {
    panic("Matrix: different size")
  }
  copy(dst.data, src.data) 
}

/******************************************************/

// Create new matrix
func New(r,c int, d []float64) *Matrix {
   if r < 0 || c < 0 {
      panic("Matrix: negative dimension!")
   }
   l := r*c
   if d == nil {
      d = make([]float64, l)
   } else if l != len(d) {
      panic("Matrix: incorrect data size!")
   }
   return &Matrix{
      rows: r,
      cols: c,
      data: d,
   }   
}

// Identity matrix
func Eye(r,c int) *Matrix {
   res := New(r,c,nil)
   n := r
   if c < r {
      n = c
   }
   shift := res.cols+1
   for i := 0; i < n; i++ {
      res.data[i*shift] = 1
   }
   return res
}

// Generate matrix with some value,
// i.e. return v*ones 
func Const(r,c int, v float64) *Matrix {
   res := New(r,c,nil)
   for i := 0; i < len(res.data); i++ {
      res.data[i] = v
   }
   return res
}


// Show matrix (debug)
func (m Matrix) Print() {
   n := 0
   for i := 0; i < m.rows; i++ {
      fmt.Println(m.data[n:n+m.cols])
      n += m.cols
   }
}

// Allocate memory for zero matrix
func emptyMatrix(r,c int) *Matrix {
   d := make([]float64, r*c)
   return &Matrix{
      rows: r,
      cols: c,
      data: d,
   }
}

// Safe getting the value
func (m Matrix) Get(r,c int) float64 {
   if m.wrongIndex(r,c) {
      panic("Matrix: wrong index!")
   }
   return *m.at(r,c)
}

// Safe setting the value
func (m Matrix) Set(r,c int, v float64) {
   if m.wrongIndex(r,c) {
      panic("Matrix: wrong index!")
   }
   *m.at(r,c) = v
}

// Multiply to constant (in place)
func (m *Matrix) Xk(k float64) *Matrix {
   for i := 0; i < len(m.data); i++ {
      m.data[i] *= k
   }
   return m
}

func (m *Matrix) Scale(k float64, src *Matrix) {
  if m.data == nil {
    m.data = make([]float64, len(src.data)) 
  }
  if len(m.data) != len(src.data) {
    panic("Matrix: different size")
  }
  for i,v := range src.data {
    m.data[i] = k * v
  }
}

/*
// Multiply two matrices
func (m *Matrix) XM(a *Matrix) *Matrix {
   if m.cols != a.rows {
      panic("Matrix: size is not compatible!")
   }
   // new size
   R,C,L := m.rows, a.cols, m.cols
   res := make([]float64,R*C)
   for r := 0; r < R; r++ {
      rC, rL := r*C, r*L
      for c := 0; c < C; c++ {
         sum := 0.0
         cc  := c
         for k := 0; k < L; k++ {
            sum += m.data[rL+k]*a.data[cc]
            cc += a.cols
         }
         res[rC+c] = sum
      }
   }
   
   return &Matrix{
      cols: C,
      rows: R,
      data: res,
   }   
}
*/ 

// Add two matrices, save result into the first one
func (a *Matrix) Add(b IMatrix) *Matrix {
   r1,c1 := a.Dim()
   r2,c2 := b.Dim()
   if r1 != r2 || c1 != c2 {
      panic("Matrix: size is not compatible!")
   }
   for r := 0; r < r1; r++ {
      for c := 0; c < c1; c++ {
         *a.at(r,c) += *b.at(r,c)
      }
   }
   return a
}

// Get differences of two matrices, save result into the first one
func (a *Matrix) Sub(b IMatrix) *Matrix {
   r1,c1 := a.Dim()
   r2,c2 := b.Dim()
   if r1 != r2 || c1 != c2 {
      panic("Matrix: size is not compatible!")
   }
   for r := 0; r < r1; r++ {
      for c := 0; c < c1; c++ {
         *a.at(r,c) -= *b.at(r,c)
      }
   }
   return a
}

// Multiply two matrices 
func Mul(a,b IMatrix) *Matrix {
   r1,c1 := a.Dim()
   r2,c2 := b.Dim()
   if c1 != r2 {
      panic("Matrix: size is not compatible!")
   }
   res := make([]float64,r1*c2)
   for r := 0; r < r1; r++ {
      rC := r*c2
      for c := 0; c < c2; c++ {
         sum := 0.0
         for k := 0; k < c1; k++ {
            sum += (*a.at(r,k)) * (*b.at(k,c))
         }
         res[rC+c] = sum         
      }
   }
   
   return &Matrix{
      cols: c2,
      rows: r1,
      data: res,
   }  
}

// Add several matrices
func Sum(mm ...IMatrix) *Matrix {
   res := mm[0].Copy()
   for i := 1; i < len(mm); i++ {
      res.Add(mm[i])
   }
   return res
}

// Multiply several matrices
func Prod(mm ...IMatrix) *Matrix {
   // start from the end to simplify vector transformation (if have)   
   N := len(mm)-1
   var res *Matrix
   if N == 0 {  // one element in matrix list
      return mm[0].Copy()
   } else {
      res = Mul(mm[N-1],mm[N])
   }
   for i := N-2; i >= 0; i-- {      
      res = Mul(mm[i],res)
   }
   return res   
}

// Vector specific methods 

// Create vector
func V(d []float64) *Matrix {
   return New(len(d), 1, d)
}

// Get vector value
func (v *Matrix) Vget(n int) float64 {
   if n >= len(v.data) {
      panic("Matrix: wrong index!")
   }
   return v.data[n]
}

func (v *Matrix) Vset(n int, q float64) {
   if n >= len(v.data) {
      panic("Matrix: wrong index!")
   }
   v.data[n] = q
}

func (v *Matrix) Norm() float64 {
   sum := 0.0
   for _,q := range v.data {
      sum += q*q
   }
   return math.Sqrt(sum)
}

func (a *Matrix) Dot(b *Matrix) float64 {
   N := len(a.data)
   if N != len(b.data) {
      panic("Matrix: not compatible!")
   }
   sum := 0.0
   for i := 0; i < N; i++ {
      sum += a.data[i] * b.data[i]
   }
   return sum
}

func (a *Matrix) Cross(b *Matrix) *Matrix {
   if len(a.data) != 3 || len(b.data) != 3 {
      panic("Matrix: vector of length 3 is expected!")
   }
   return &Matrix {
             rows: 3,
             cols: 1,
             data: []float64{
                a.data[1]*b.data[2]-b.data[1]*a.data[2],
                a.data[2]*b.data[0]-a.data[0]*b.data[2],
                a.data[0]*b.data[1]-b.data[0]*a.data[1]},
          }
}
