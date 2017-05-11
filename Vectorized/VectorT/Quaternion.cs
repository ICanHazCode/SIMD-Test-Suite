using System;
using System.Collections.Generic;
using System.Text;
using System.Numerics;
namespace Vectorized.VectorT
{
	//TODO: Am I doing this wrong ?!
	// It seems almost as if the Vector<T> struct is for direct register maipulation
	//  and should only be temporary, storing the data in an array(for quick reload).
	/// <summary>
	/// This Quaternion is made from the Vector T Template.
	/// It is immutable (readonly)
	/// </summary>
	/// <remarks>This immutability will require more uses of new than</remarks>
	public struct Quaternion : IEquatable<Quaternion>
	{
		// 4 floats can be saved in the register
		// This holds the 4 as an array
		public float[] Q;

		public static Quaternion Identity => _Identity;
		public float X => Q[0];
		public float Y => Q[1];
		public float Z => Q[2];
		public float W => Q[3];

		public Quaternion(float x, float  y, float z, float w)
		{
			Q =  new float[]{ x, y, z, w };
		}
		public Quaternion(float[] v)
		{
			Q = v;
		}
		public Quaternion(Vector<float> v):this( v[0], v[1], v[2], v[3] )
		{ }

		public Quaternion(Vector3 v, float w) : this(v.X, v.Y, v.Z, w) { }

		public static Quaternion CreateFromAxisAngle(Vector3 axis, float angle)
		{
			float num = angle * 0.5f;
			float num2 = (float)Math.Sin((double)num);
			float w = (float)Math.Cos((double)num);
			var v = axis * num2;
			//result.X = axis.X * num2;
			//result.Y = axis.Y * num2;
			//result.Z = axis.Z * num2;
			return new Quaternion(v, w);
		}

		public static Quaternion Add(Quaternion left, Quaternion right)
		{
			return new Quaternion(new Vector<float>(left.Q) + new Vector<float>(right.Q));
		}

		public static Quaternion Subtract(Quaternion left, Quaternion right)
		{
			return new Quaternion(new Vector<float>(left.Q) - new Vector<float>(right.Q));
		}

		public static Quaternion operator +(Quaternion left, Quaternion right)
		{
			return Add(left, right);
		}
		public static Quaternion operator -(Quaternion left, Quaternion right)
		{
			return Subtract(left, right);
		}

		public static Quaternion Multiply(Quaternion left, Quaternion right)
		{
			// Don't know if swizzling works
			// uses 7 'registers'. 8 available on x86, 16 available on x64
			var Qr = new Vector<float>(right.Q);
			var Ql = new Vector<float>(left.Q);
			var nQr = -Qr;
			//Swizzles!
			var vX = Vector.Dot(Ql, new Vector<float>(new float[] { Qr[3], Qr[2], nQr[1], Qr[0] }));
			var vY = Vector.Dot(Ql, new Vector<float>(new float[] { nQr[2], Qr[3], Qr[0], Qr[1] }));
			var vZ = Vector.Dot(Ql, new Vector<float>(new float[] { Qr[1], nQr[0], Qr[3], Qr[2] }));
			var vW = Vector.Dot(Ql, new Vector<float>(new float[] { nQr[0], nQr[1], nQr[2], Qr[3] }));
			//result.X = x * bW + y * bZ + z *-bY + w * bX;
			//result.Y = x *-bZ + y * bW + z * bX + w * bY;
			//result.Z = x * bY + y *-bX + z * bW + w * bZ;
			//result.W = x *-bX + y *-bY + z *-bZ + w * bW;

			return new Quaternion(vX, vY, vZ, vW);
		}

		/// <summary>
		/// Scales a quaternion.
		/// </summary>
		/// <param name="q">Quaternion to multiply.</param>
		/// <param name="scale">Amount to multiply each component of the quaternion by.</param>
		/// <param name="result">Scaled quaternion.</param>
		public static Quaternion Scale(Quaternion q, float scale)
		{
			return new Quaternion(new Vector<float>(q.Q) * scale);
		}

		public static Quaternion operator *(Quaternion a, float b)
		{
			return Scale(a, b);
		}

		public static float Dot(Quaternion a, Quaternion b)
		{
			return Vector.Dot(new Vector<float>(a.Q), new Vector<float>(b.Q));
		}

		public static Quaternion Divide(Quaternion value1, Quaternion value2)
		{
			var v1 = new Vector<float>(value1.Q);
			Vector<float> v3;
			{
				var v2 = new Vector<float>(value2.Q);
				float num2 = 1f / Vector.Dot(v2, v2);
				v3 = new Vector<float>(Conjugate(value2).Q) * num2;
			}
			Quaternion r;
			{
				var nv3 = -v3;
				var v = new Vector<float>(v1[3]) * v3;
				v += (new Vector<float>(v1[2]) * new Vector<float>(new float[] { nv3[1], v3[0], v3[3], nv3[2] }));
				v += (new Vector<float>(v1[1]) * new Vector<float>(new float[] { v3[2], v3[3], nv3[0], nv3[1] }));
				v += (new Vector<float>(v1[0]) * new Vector<float>(new float[] { v3[3], nv3[2], v3[1], nv3[0] }));
				r.Q = new float[] { v[0], v[1], v[2], v[3] };
				//             v1        v2         v3          v4
				//result.X = x*v3[3]  + y*v3[2]  + z*-v3[1] + w*v3[0];//num7;
				//result.Y = x*-v3[2] + y*v3[3]  + z*v3[0]  + w*v3[1];//num8;
				//result.Z = x*v3[1]  + y*-v3[0] + z*v3[3]  + w*v3[2];//num9;
				//result.W = x*-v3[0] + y*-v3[1] + z*-v3[2] + w*v3[3];//num10;
				//
			}

			return r;
		}
		public static Quaternion operator /(Quaternion l,Quaternion r)
		{
			return Divide(l, r);
		}

		public static Quaternion Concatenate(Quaternion a, Quaternion b)
		{
			//Uses 7 'Registers'
			var Ql = new Vector<float>(a.Q);
			var Qr = new Vector<float>(b.Q);
			var nQr = -Qr;

			//Swizzles!
			var vX = Vector.Dot(Ql, new Vector<float>(new float[] {  Qr[3], nQr[2],  Qr[1], Qr[0] }));
			var vY = Vector.Dot(Ql, new Vector<float>(new float[] {  Qr[2],  Qr[3], nQr[0], Qr[1] }));
			var vZ = Vector.Dot(Ql, new Vector<float>(new float[] { nQr[1],  Qr[0],  Qr[3], Qr[2] }));
			var vW = Vector.Dot(Ql, new Vector<float>(new float[] { nQr[0], nQr[1], nQr[2], Qr[3] }));
			//result.X = aX * bW + aY *-bZ + aZ * bY + aW * bX;
			//result.Y = aX * bZ + aY * bW + aZ *-bX + aW * bY;
			//result.Z = aX *-bY + aY * bX + aZ * bW + aW * bZ;
			//result.W = aX *-bX + aY *-bY + aZ *-bZ + aW * bW;
			return new Quaternion(vX,vY,vZ,vW);
		}

		public static Quaternion Normalize(Quaternion quaternion)
		{
			var q = new Vector<float>(quaternion.Q);
			//var inverse = Vector<float>.One / Vector.SquareRoot(new Vector<float>(Vector.Dot(q,q))) ;
			return new Quaternion(q / Vector.SquareRoot(new Vector<float>(Vector.Dot(q, q))));
		}
		public float LengthSquared()
		{
			var q = new Vector<float>(Q);
			return Vector.Dot(q, q);
		}

		public float Length()
		{
			var q = new Vector<float>(Q);
			var qq = new Vector<float>(Vector.Dot(q, q));
			return Vector.SquareRoot(qq)[0];
		}

		public static Quaternion Slerp(Quaternion start, Quaternion end, float interpolationAmount)
		{
			//TODO: more register work
			// double cosHalfTheta = start.W * end.W + start.X * end.X + start.Y * end.Y + start.Z * end.Z;
			var e = new Vector<float>(end.Q);
			var s = new Vector<float>(start.Q);

			var cosHalfTheta = Vector.Dot(s, e);
			if (cosHalfTheta < 0)
			{
				//Negating a quaternion results in the same orientation, 
				e = -e;
				//but we need cosHalfTheta to be positive to get the shortest path.
				cosHalfTheta = -cosHalfTheta;
			}
			// If the orientations are similar enough, then just pick one of the inputs.
			if (cosHalfTheta > (1.0f - 1e-12f))
			{
				return end;
			}
			// Calculate temporary values.
			// No available SIMD maths for these functions yet!
			double halfTheta = Math.Acos(cosHalfTheta);
			double sinHalfTheta = Math.Sqrt(1.0 - cosHalfTheta * cosHalfTheta);

			double aFraction = Math.Sin((1 - interpolationAmount) * halfTheta) / sinHalfTheta;
			double bFraction = Math.Sin(interpolationAmount * halfTheta) / sinHalfTheta;

			//Blend the two quaternions to get the result!
			var va = new Vector<float>((float)aFraction);
			var vb = new Vector<float>((float)bFraction);
			return new Quaternion((s * va) + (e * vb));
			//result.X = (float)(start.X * aFraction + end.X * bFraction);
			//result.Y = (float)(start.Y * aFraction + end.Y * bFraction);
			//result.Z = (float)(start.Z * aFraction + end.Z * bFraction);
			//result.W = (float)(start.W * aFraction + end.W * bFraction);
			
		}

		public static Quaternion Conjugate(Quaternion quaternion)
		{
			Quaternion q;
			q.Q = new float[4] { -quaternion.Q[0], -quaternion.Q[1], -quaternion.Q[2], quaternion.Q[3] };
			return q;
		}

		public static Quaternion Inverse(Quaternion quaternion)
		{
			var q = new Vector<float>(quaternion.Q);
			var nq = -q;
			return new Quaternion(new Vector<float>(new float[] { nq[0], nq[1], nq[2], q[3] }) * Vector.Dot(q,q));
		}
		public static bool operator ==(Quaternion a, Quaternion b)
		{
			return (new Vector<float>(a.Q) == new Vector<float>(b.Q));
		}
		public static bool operator !=(Quaternion a, Quaternion b)
		{
			return !(a == b);
		}

		public static Quaternion Negate(Quaternion q)
		{
			Quaternion r;
			r.Q = new float[4] { -q.Q[0], -q.Q[1], -q.Q[2], -q.Q[3] };
			return r;
		}
		public static Quaternion operator -(Quaternion q)
		{
			Quaternion r;
			r.Q = new float[4] { -q.Q[0], -q.Q[1], -q.Q[2], -q.Q[3] };
			return r;
		}

		public bool Equals(Quaternion other)
		{
			return (this == other);
		}

		public override bool Equals(object obj)
		{
			if (obj is Quaternion)
				return Equals((Quaternion)obj);
			return false;
		}

		public override int GetHashCode()
		{
			return new Vector<float>(Q).GetHashCode();
		}

		public static Vector3 Transform(ref Vector3 v, ref Quaternion rotation)
		{
			// Vectorized ... Unfortunately, I expect some spillage on x86 :(
			Vector3 result;
			//1
			var vin = new Vector<float>(new float[] { v.X, v.Y, v.Z, 0f });

			Vector<float> xyz2;
			Vector<float> xxyyzz2;
			Vector<float> wx2xy2xz2;
			Vector<float> wy2yz2wz2;
			//5
			{
				//I expect spillage on x86 :/
				//6
				var rot = new Vector<float>(rotation.Q);
				xyz2 = rot + rot;
				xxyyzz2 = rot * xyz2;
				//7
				var yzz2 = new Vector<float>(new float[] { xyz2[1], xyz2[2], xyz2[2], 0f });
				//8
				var wyw = new Vector<float>(new float[] { rot[3], rot[1], rot[3], 0f });

				wx2xy2xz2 = new Vector<float>(new float[] { rot[3], rot[0], rot[0], 0f }) * xyz2;
				wy2yz2wz2 = wyw * yzz2;
			}
			//5
			//float transformedX = v.X * (1f - yy2 - zz2) + v.Y * (xy2 - wz2) + v.Z * (xz2 + wy2);
			//float transformedY = v.X * (xy2 + wz2) + v.Y * (1f - xx2 - zz2) + v.Z * (yz2 - wx2);
			//float transformedZ = v.X * (xz2 - wy2) + v.Y * (yz2 + wx2) + v.Z * (1f - xx2 - yy2);
			//6
			Vector<float> v1;
			{
				//7
				var vX = new Vector<float>(vin[0]);
				//     X               Y                Z            w
				//v1 vX*1f+(-yy2)+(-zz2)   vX*0f+xy2+wz2    vX*0f+xz2+(-wy2)  0
				//   t1   t2    t3
				// X 1F  -yy2  -zz2
				// Y 0F  xy2   wz2
				// Z 0F  xz2   -wy2
				// W 0    0     0
				//8
				var t1 = new Vector<float>(new float[]
			   {
				Vector<float>.One[0],
				Vector<float>.Zero[1],
				Vector<float>.Zero[2],
				Vector<float>.Zero[3]
			   });
				//9
				var t2 = new Vector<float>(new float[]
				{
				-xyz2[1],
				wx2xy2xz2[1],
				wx2xy2xz2[2],
				Vector<float>.Zero[3]
				});
				//10
				var t3 = new Vector<float>(new float[]
				{
				-xyz2[2],
				wy2yz2wz2[2],
				-wy2yz2wz2[0],
				Vector<float>.Zero[3]
				});
				v1 = (t1 + t2 + t3) * vX;
			}
			//7
			Vector<float> v2;
			{
				var vY = new Vector<float>(vin[1]);
				//v2 vY*0f+xy2+(-wz2)   vY*1f+(-xx2)+(-zz2)    vY*0f+yz2+wx2  0
				//    t1	t2    t3
				// X  0F   xy2   -wz2
				// Y  1F  -xx2   -zz2
				// Z  0F   yz2    wx2
				// W  0F   0F     0F
				var t1 = new Vector<float>(new float[]
				{
				Vector<float>.Zero[0],
				Vector<float>.One[1],
				Vector<float>.Zero[2],
				Vector<float>.Zero[3]
				});
				var t2 = new Vector<float>(new float[]
				{
				wx2xy2xz2[1],
				-xyz2[0],
				wy2yz2wz2[1],
				Vector<float>.Zero[3]
				});
				var t3 = new Vector<float>(new float[]
				{
				-wy2yz2wz2[2],
				-xyz2[2],
				wx2xy2xz2[0],
				Vector<float>.Zero[3]
				});
				//11
				v2 = (t1 + t2 + t3) * vY;
			}
			//8
			Vector<float> v3;
			{
				var vZ = new Vector<float>(vin[2]);
				//v3 vZ*0f+xz2+wy2   vZ*0f+yz2+(-wx2)    vZ*1f+(-xx2)+(-yy2)  0
				//    t1	t2    t3
				// X  0F   xz2    wy2
				// Y  0F   yz2   -wx2
				// Z  1F  -xx2   -yy2
				// W  0F   0F     0F
				var t1 = new Vector<float>(new float[]
				{
				Vector<float>.Zero[0],
				Vector<float>.Zero[1],
				Vector<float>.One[2],
				Vector<float>.Zero[3]
				});
				var t2 = new Vector<float>(new float[]
				{
				wx2xy2xz2[2],
				wy2yz2wz2[1],
				-xyz2[0],
				Vector<float>.Zero[3]
				});
				var t3 = new Vector<float>(new float[]
				{
				wy2yz2wz2[0],
				-wx2xy2xz2[0],
				-xyz2[1],
				Vector<float>.Zero[3]
				});
				//12
				v3 = (t1 + t2 + t3) * vZ;
			}
			//9
			var vr = v1 + v2 + v3;
			result.X = vr[0];
			result.Y = vr[1];
			result.Z = vr[2];
			return result;

		}


		/// <summary>
		/// Vectorized version of the Dot Dot Cross vector Transform
		/// </summary>
		/// <param name="v"></param>
		/// <param name="q"></param>
		/// <returns></returns>
		public static Vector3 FastTransform(Vector3 v, Quaternion q)
		{
			//void rotate_vector_by_quaternion(const Vector3&v, const Quaternion&q, Vector3 & vprime)
			//{
			//	// Extract the vector part of the quaternion
			//	Vector3 u(q.x, q.y, q.z);

			//	// Extract the scalar part of the quaternion
			//	float s = q.w;

			//	// Do the math
			//	vprime = 2.0f * dot(u, v) * u
			//		  + (s * s - dot(u, u)) * v
			//		  + 2.0f * s * cross(u, v);
			//}
			float s = q.Q[3];
			Vector<float> u = new Vector<float>(new float[] { q.Q[0], q.Q[1], q.Q[2], 0.0f });
			Vector<float> v2 = new Vector<float>(new float[] { v.X, v.Y, v.Z, 0f });
			var d1 = Vector.Dot(u, v2);
			var d2 = Vector.Dot(u, u);
			//var r = 2.0f * Vector3.Dot(u, v) * u 
			//		+ 2.0f * s * Vector3.Cross(u, v)
			//		+ (s * s - Vector3.Dot(u, u)) * v;
			//Vector3 Cross(vector1, vector2) = 
			//         t1          t2          t3          t4
			// X =  vector1.Y * vector2.Z - vector1.Z * vector2.Y, 
			// Y =  vector1.Z * vector2.X - vector1.X * vector2.Z,
			// Z =  vector1.X * vector2.Y - vector1.Y * vector2.X
			var t1 = new Vector<float>(new float[] { u[1], u[2], u[0], 0f });
			var t2 = new Vector<float>(new float[] { u[2], u[0], u[1], 0f });
			// -
			var t3 = new Vector<float>(new float[] { u[2], u[0], u[1], 0f });
			var t4 = new Vector<float>(new float[] { u[1], u[2], u[0], 0f });
			// =
			var cp = (t1 * t2) - (t3 * t4);

			t1 = u * (d1* 2.0f);
			t2 = cp * (s * 2.0f);
			t3 = v2 * (s * (s - d2));
			var r = t1 + t2 + t3;
			return new Vector3(r[0],r[1],r[2]);
		}

		public static Quaternion operator *(Quaternion a, Quaternion b)
		{
			return Multiply(a, b);
		}
		public static float GetAngleFromQuaternion(ref Quaternion q)
		{
			float qw = Math.Abs(q.W);
			if (qw > 1)
				return 0;
			return 2 * (float)Math.Acos(qw);
		}
		/// <summary>
		/// Computes the rotation from the start orientation to the end orientation such that end = Quaternion.Concatenate(start, relative).
		/// </summary>
		/// <param name="start">Starting orientation.</param>
		/// <param name="end">Ending orientation.</param>
		/// <param name="relative">Relative rotation from the start to the end orientation.</param>
		public static Quaternion GetRelativeRotation(ref Quaternion start, ref Quaternion end)
		{
			Quaternion startInverse =	Conjugate(start);
			return Concatenate( startInverse, end);
		}


		/// <summary>
		/// Transforms the rotation into the local space of the target basis such that rotation = Quaternion.Concatenate(localRotation, targetBasis)
		/// </summary>
		/// <param name="rotation">Rotation in the original frame of reference.</param>
		/// <param name="targetBasis">Basis in the original frame of reference to transform the rotation into.</param>
		/// <param name="localRotation">Rotation in the local space of the target basis.</param>
		public static Quaternion GetLocalRotation( Quaternion rotation, Quaternion targetBasis)
		{
			Quaternion basisInverse = Conjugate( targetBasis);
			return Concatenate(rotation, basisInverse);
		}

		/// <summary>
		/// Gets a string representation of the quaternion.
		/// </summary>
		/// <returns>String representing the quaternion.</returns>
		public override string ToString()
		{
			return "{ X: " + X + ", Y: " + Y + ", Z: " + Z + ", W: " + W + "}";
		}

		private static Quaternion _Identity = new Quaternion ( 0f, 0f, 0f, 1f );
		

    }
}
