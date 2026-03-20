package geometry

import "server2/math/fixed"

type Triangle struct {
	A Vector3
	B Vector3
	C Vector3
}

func NewTriangle(a, b, c Vector3) Triangle {
	return Triangle{A: a, B: b, C: c}
}

func (t Triangle) EdgeAB() Vector3 {
	return t.B.Sub(t.A)
}

func (t Triangle) EdgeAC() Vector3 {
	return t.C.Sub(t.A)
}

func (t Triangle) EdgeBC() Vector3 {
	return t.C.Sub(t.B)
}

func (t Triangle) UnnormalizedNormal() Vector3 {
	return t.EdgeAB().Cross(t.EdgeAC())
}

func (t Triangle) Normal() Vector3 {
	n := t.UnnormalizedNormal()
	return n.Normalize()
}

// ClosestPoint returns the closest point on triangle to p.
// Based on Real-Time Collision Detection (Christer Ericson).
func (t Triangle) ClosestPointTo(p Vector3) Vector3 {
	ab := t.B.Sub(t.A)
	ac := t.C.Sub(t.A)
	ap := p.Sub(t.A)

	d1 := ab.Dot(ap)
	d2 := ac.Dot(ap)
	if d1.Cmp(fixed.Zero) <= 0 && d2.Cmp(fixed.Zero) <= 0 {
		return t.A
	}

	bp := p.Sub(t.B)
	d3 := ab.Dot(bp)
	d4 := ac.Dot(bp)
	if d3.Cmp(fixed.Zero) >= 0 && d4.Cmp(d3) <= 0 {
		return t.B
	}

	vc := d1.Mul(d4).Sub(d3.Mul(d2))
	if vc.Cmp(fixed.Zero) <= 0 && d1.Cmp(fixed.Zero) >= 0 && d3.Cmp(fixed.Zero) <= 0 {
		denom := d1.Sub(d3)
		if denom == fixed.Zero {
			return t.A
		}
		v := d1.Div(denom)
		return t.A.Add(ab.Scale(v))
	}

	cp := p.Sub(t.C)
	d5 := ab.Dot(cp)
	d6 := ac.Dot(cp)
	if d6.Cmp(fixed.Zero) >= 0 && d5.Cmp(d6) <= 0 {
		return t.C
	}

	vb := d5.Mul(d2).Sub(d1.Mul(d6))
	if vb.Cmp(fixed.Zero) <= 0 && d2.Cmp(fixed.Zero) >= 0 && d6.Cmp(fixed.Zero) <= 0 {
		denom := d2.Sub(d6)
		if denom == fixed.Zero {
			return t.A
		}
		w := d2.Div(denom)
		return t.A.Add(ac.Scale(w))
	}

	va := d3.Mul(d6).Sub(d5.Mul(d4))
	if va.Cmp(fixed.Zero) <= 0 &&
		d4.Sub(d3).Cmp(fixed.Zero) >= 0 &&
		d5.Sub(d6).Cmp(fixed.Zero) >= 0 {
		denom := d4.Sub(d3).Add(d5.Sub(d6))
		if denom == fixed.Zero {
			return t.B
		}
		bc := t.C.Sub(t.B)
		w := d4.Sub(d3).Div(denom)
		return t.B.Add(bc.Scale(w))
	}

	denom := va.Add(vb).Add(vc)
	if denom == fixed.Zero {
		return t.A
	}

	v := vb.Div(denom)
	w := vc.Div(denom)

	return t.A.Add(ab.Scale(v)).Add(ac.Scale(w))
}
