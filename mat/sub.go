package mat 
/* Access to modified matrix elements without real source modification */

/****************** Transposed matrix ******************/

// Pointer to the transposed matrix elements.
type Transposed struct {
   m *Matrix
}

// Interface methods 

// Access to the matrix element.
func (t Transposed) at(r,c int) *float64 {
   return t.m.at(c,r)
}

// Check index correctness.
func (t Transposed) wrongIndex(r,c int) bool {
   return t.m.wrongIndex(c,r)
}

// Return height and width of the matrix.
func (t Transposed) Dim() (r,c int) {
   return t.m.cols, t.m.rows
}

// Return copy as a new matrix object.
func (t Transposed) Copy() *Matrix {
   res := emptyMatrix(t.m.cols, t.m.rows)
   for r := 0; r < res.rows; r++ {
      for c := 0; c < res.cols; c++ {
         *res.at(r,c) = *t.m.at(c,r)
      }
   }
   return res
}

// Get transposed matrix. 
func (m Matrix) T() *Transposed {
   return &Transposed {
      m: &m,
   }
}

/***************** Part of matrix ******************/

// Access to the matrix blocks.
type Submatrix struct {
   m *Matrix
   r0, c0 int     // start position
   nr, nc int     // number of elements
}

// Interface methods 

// Check element index.
func (s Submatrix) wrongIndex(r,c int) bool {
   return r < 0 || r >= s.nr || c < 0 || c >= s.nc
}

// Access to the matrix element.
func (s Submatrix) at(r,c int) *float64 {
   return s.m.at(s.r0+r,s.c0+c)
}

// Return number of rows and columns.
func (s Submatrix) Dim() (r,c int) {
   return s.nr, s.nc
}

// Return copy as a new matrix.
func (s Submatrix) Copy() *Matrix {
   res := emptyMatrix(s.nr, s.nc)
   for r := 0; r < s.nr; r++ {
      for c := 0; c < s.nc; c++ {
         *res.at(r,c) = *s.m.at(s.r0+r,s.c0+c)
      }
   }
   return res
}

// Define submatrix as an initial position and width/height. 
func (m Matrix) Block(r,c,Nr,Nc int) *Submatrix {
   if m.wrongIndex(r,c) || m.wrongIndex(r+Nr-1,c+Nc-1) {
      panic("Matrix: wrong size!")
   }
   return &Submatrix{
      m: &m,
      r0: r,
      c0: c,
      nr: Nr,
      nc: Nc,
   }
}

// Get matrix column.
func (m Matrix) Col(c int) *Submatrix {
   return m.Block(0,c,m.rows,1)
}

// Get matrix row.
func (m Matrix) Row(r int) *Submatrix {
   return m.Block(r,0,1,m.cols)
}

// Copy values into specified region of matrix from any available object. 
func (s *Submatrix) Insert(v IMatrix) *Submatrix {
   rv,cv := v.Dim()
   if s.nr != rv || s.nc != cv {
      panic("Matrix: size is not compatible!")
   }
   for r := 0; r < rv; r++ {
      for c := 0; c < cv; c++ {
         *s.at(r,c) = *v.at(r,c)
      }
   }
   return s
}
