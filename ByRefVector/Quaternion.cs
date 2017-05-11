
using System;
using System.Globalization;
using System.Numerics;
namespace ByRefVector
{
	/// <summary>
	/// This Quaternion is made from a Vector4
	/// </summary>
	public struct Quaternion
	{
		public Vector4 Q;
		public static Quaternion Identity => _Identity;
		public float X => Q.X;
		public float Y => Q.Y;
		public float Z => Q.Z;
		public float W => Q.W;

		public Quaternion(float x, float y, float z, float w)
		{
			Q.X = x;
			Q.Y = y;
			Q.Z = z;
			Q.W = w;
		}
		public Quaternion(Vector4 v)
		{
			Q = v;
		}

		public Quaternion(Vector3 v, float w) : this(v.X, v.Y, v.Z, w)
		{ }

		public static Quaternion CreateFromAxisAngle(Vector3 axis, float angle)
		{
			float num = angle * 0.5f;
			float num2 = (float)Math.Sin((double)num);
			float w = (float)Math.Cos((double)num);
			var result = axis * num2;
			//Vector3 result;
			//result.X = axis.X * num2;
			//result.Y = axis.Y * num2;
			//result.Z = axis.Z * num2;
			return new Quaternion(result, w);
		}


		public static void Add(ref Quaternion left,ref Quaternion right,out Quaternion ret)
		{
			ret.Q = (left.Q + right.Q);
		}

		public static void Subtract(ref Quaternion left,ref Quaternion right,out Quaternion ret)
		{
			ret.Q = (left.Q - right.Q);
		}

		public static Quaternion operator +(Quaternion left, Quaternion right)
		{
			Quaternion ret;
			Add(ref left,ref right,out ret);
			return ret;
		}
		public static Quaternion operator -(Quaternion left, Quaternion right)
		{
			Quaternion ret;
			Subtract(ref left,ref right,out ret);
			return ret;
		}

		public static void Multiply(ref Quaternion left,ref Quaternion right,out Quaternion result)
		{
			// Don't know if swizzling works
			var Qr = right.Q;
			var Ql = left.Q;
			var nQr = -Qr;
			//Swizzles!
			{
				Vector4 t;
				t.X = Qr.W;
				t.Y = Qr.Z;
				t.Z = nQr.Y;
				t.W = Qr.X;
				result.Q.X = Vector4.Dot(Ql, t);
				t.X = nQr.Z;
				t.Y = Qr.W;
				t.Z = Qr.X;
				t.W = Qr.Y;
				result.Q.Y = Vector4.Dot(Ql, t);
				t.X = Qr.Y;
				t.Y = nQr.X;
				t.Z = Qr.W;
				t.W = Qr.Z;
				result.Q.Z = Vector4.Dot(Ql, t);
				t.X = nQr.X;
				t.Y = nQr.Y;
				t.Z = nQr.Z;
				t.W = Qr.W;
				result.Q.W = Vector4.Dot(Ql, t);
			}
			//result.X = x * bW + y * bZ + z *-bY + w * bX;
			//result.Y = x *-bZ + y * bW + z * bX + w * bY;
			//result.Z = x * bY + y *-bX + z * bW + w * bZ;
			//result.W = x *-bX + y *-bY + z *-bZ + w * bW;

			//return result;
		}

		/// <summary>
		/// Scales a quaternion.
		/// </summary>
		/// <param name="q">Quaternion to multiply.</param>
		/// <param name="scale">Amount to multiply each component of the quaternion by.</param>
		/// <param name="r">Scaled quaternion.</param>
		public static void Scale(ref Quaternion q, float scale,out Quaternion r)
		{
			r.Q = q.Q * scale;
		}
		public static Quaternion operator *(Quaternion a, float b)
		{
			Quaternion r;
			Scale(ref a,b,out r);
			return r;
		}
		public static void Divide(ref Quaternion value1,ref Quaternion value2,out Quaternion r)
		{
			var v1 = value1.Q;
			Quaternion v3;
			{
				var v2 = value2.Q;
				float num2 = 1f / Vector4.Dot(v2, v2);
				Conjugate(ref value2,out v3);
				v3 *= num2;
			}
//			Quaternion r;
			{
				var nv3 = -v3.Q;
				//float x = value1.X;
				//float y = value1.Y;
				//float z = value1.Z;
				//float w = value1.W;
				//float num = value2.X * value2.X + value2.Y * value2.Y + value2.Z * value2.Z + value2.W * value2.W;
				//float num2 = 1f / num;
				//float num3 = -value2.X * num2;
				//float num4 = -value2.Y * num2;
				//float num5 = -value2.Z * num2;
				//float num6 = value2.W * num2;
				//n v3.X = num3
				//n v3.Y = num4
				//n v3.Z = num5
				//n v3.W = num6
				//float num7 = y*v3.Z + z*nv3.Y;
				//float num8 = z*v3.X + x*nv3.Z;
				//float num9 = x*v3.Y + y*-v3.X;
				//float num10 = x*nv3.X + y*nv3.Y + z*nv3.Z;
				//Quaternion result;
				//             v        v1       v2        v3
				//result.X = x*v3.W  + y*v3.Z  + z*nv3.Y + v3.X*w;
				//result.Y = x*nv3.Z + y*v3.W  + z*v3.X  + v3.Y*w; num8;
				//result.Z = x*v3.Y  + y*nv3.X + z*v3.W  + v3.Z*w; num9;
				//result.W = x*nv3.X + y*nv3.Y + z*nv3.Z + v3.W*w;
				var v = new Vector4(v3.W, nv3.Z, v3.Y, nv3.X) * v1.X;
				v = v + (new Vector4(v3.Z, v3.W, nv3.X, nv3.Y) * v1.Y);
				v = v + (new Vector4(nv3.Y, v3.X, v3.W, nv3.Z) * v1.Z);
				v = v + (v3.Q * v1.W);
				r.Q = v;
				//return result;

				//             v1        v2         v3          v4
				//result.X = x*v3[3]  + y*v3[2]  + z*-v3[1] + w*v3[0];//num7;
				//result.Y = x*-v3[2] + y*v3[3]  + z*v3[0]  + w*v3[1];//num8;
				//result.Z = x*v3[1]  + y*-v3[0] + z*v3[3]  + w*v3[2];//num9;
				//result.W = x*-v3[0] + y*-v3[1] + z*-v3[2] + w*v3[3];//num10;
				//
			}

	//		return r;
		}
		public static Quaternion operator /(Quaternion l, Quaternion r)
		{
			Quaternion ret;
			Divide(ref l,ref r,out ret);
			return ret;
		}


		public static void Concatenate(ref Quaternion a,ref Quaternion b,out Quaternion r)
		{
			Vector4 v = new Vector4(b.X, b.Y, b.Z, b.W);
			Vector4 v2 = new Vector4(a.X, a.Y, a.Z, a.W);

			//v.X = b.X;
			//v.Y = b.Y;
			//v.Z = b.Z;
			//v.W = b.W;
			//v2.X = a.X;
			//v2.Y = a.Y;
			//v2.Z = a.Z;
			//v2.W = a.W;
			//float x = b.X;
			//float y = b.Y;
			//float z = b.Z;
			//float w = b.W;
			//float x2 = a.X;
			//float y2 = a.Y;
			//float z2 = a.Z;
			//float w2 = a.W;
			float num = v.Y * v2.Z - v.Z * v2.Y;
			float num2 = v.Z * v2.X - v.X * v2.Z;
			float num3 = v.X * v2.Y - v.Y * v2.X;
			float num4 = v.X * v2.X + v.Y * v2.Y + v.Z * v2.Z;
			//Quaternion result;
			r.Q.X = v.X * v2.W + v2.X * v.W + num;
			r.Q.Y = v.Y * v2.W + v2.Y * v.W + num2;
			r.Q.Z = v.Z * v2.W + v2.Z * v2.W + num3;
			r.Q.W = v.W * v2.W - num4;
			//return result;

//			//Uses 7 'Registers'
//			var Ql = a.Q;
//			var Qr = b.Q;
//			var nQr = -Qr;
////			Quaternion r;
//			//Swizzles!
//			{
//				Vector4 t;
//				t.X = Qr.W;
//				t.Y = nQr.Z;
//				t.Z = Qr.Y;
//				t.W = Qr.X;
//				r.Q.X = Vector4.Dot(Ql, t);
//				t.X = Qr.Z;
//				t.Y = Qr.W;
//				t.Z = nQr.X;
//				t.W = Qr.Y;
//				r.Q.Y = Vector4.Dot(Ql, t);
//				t.X = nQr.Y;
//				t.Y = Qr.X;
//				t.Z = Qr.W;
//				t.W = Qr.Z;
//				r.Q.Z = Vector4.Dot(Ql, t);
//				t.X = nQr.X;
//				t.Y = nQr.Y;
//				t.Z = nQr.Z;
//				t.W = Qr.W;
//				r.Q.W = Vector4.Dot(Ql, t);
//			}
//			//result.X = aX * bW + aY *-bZ + aZ * bY + aW * bX;
//			//result.Y = aX * bZ + aY * bW + aZ *-bX + aW * bY;
//			//result.Z = aX *-bY + aY * bX + aZ * bW + aW * bZ;
//			//result.W = aX *-bX + aY *-bY + aZ *-bZ + aW * bW;
//	//		return r;
		}

		public static void Normalize(ref Quaternion quaternion,out Quaternion r)
		{
			//Quaternion r;
			var q = quaternion.Q;
			//var inverse = Vector<float>.One / Vector.SquareRoot(new Vector<float>(Vector.Dot(q,q))) ;
			r.Q = q / (float)Math.Sqrt(Vector4.Dot(q, q));
			//return r;
		}
		public float LengthSquared()
		{
			return Vector4.Dot(Q,Q);
		}

		public float Length()
		{
			var d = new Vector4(Vector4.Dot(Q, Q));
			return Vector4.SquareRoot(d).X;
		}

		public static void Slerp(ref Quaternion start,ref Quaternion end, float interpolationAmount,out Quaternion r)
		{
			var e = end.Q;
			var s = start.Q;
		//	Quaternion r;
			var cosHalfTheta = Vector4.Dot(s, e);
			bool flag = false;
			if (cosHalfTheta < 0)
			{
				cosHalfTheta = -cosHalfTheta;
				flag = true;
			}
			double aFraction, bFraction;
			if (cosHalfTheta > 0.999999f)
			{
				aFraction = 1f - interpolationAmount;
				bFraction = (flag ? (-interpolationAmount) : interpolationAmount);
			}
			else
			{
				double halfTheta = Math.Acos(cosHalfTheta);
				double sinHalfTheta = 1.0 / Math.Sin(halfTheta);
				aFraction = Math.Sin((1 - interpolationAmount) * halfTheta) * sinHalfTheta;
				bFraction = (flag ? -Math.Sin(interpolationAmount * halfTheta) * sinHalfTheta
					: Math.Sin(interpolationAmount * halfTheta) * sinHalfTheta);
			}
			var va = (float)aFraction;
			var vb = (float)bFraction;
			r.Q = ((s * va) + (e * vb));
			//return r;
		}

		public static void Conjugate(ref Quaternion quaternion,out Quaternion q)
		{
//			Quaternion q;
			q.Q.X = -quaternion.Q.X;
			q.Q.Y = -quaternion.Q.Y;
			q.Q.Z = -quaternion.Q.Z;
			q.Q.W = quaternion.Q.W;
	//		return q;
		}

		public static void Inverse(ref Quaternion quaternion,out Quaternion r)
		{
			var q = quaternion.Q;
			Conjugate(ref quaternion, out r);
			Scale(ref r, 1f / Vector4.Dot(q, q),out r);
		}
		public static bool operator ==(Quaternion a, Quaternion b)
		{
			return (a.Q == b.Q);
		}
		public static bool operator !=(Quaternion a, Quaternion b)
		{
			return !(a == b);
		}

		public static void Negate(ref Quaternion q, out Quaternion r)
		{
//			Quaternion r;
			r.Q = -q.Q;
//			return r;
		}
		public static Quaternion operator -(Quaternion q)
		{
			Quaternion r;
			r.Q = -q.Q;
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
			return Q.X.GetHashCode() + Q.Y.GetHashCode() + Q.Z.GetHashCode() + Q.W.GetHashCode();
		}

		//Long Form
		public static void TransformLong(ref Vector3 v,ref Quaternion rotation,out Vector3 r)
		{
			Vector4 vin = new Vector4(v.X, v.Y, v.Z, 0f);
			Vector4 xxyyzz2;
			Vector4 wx2xy2xz2;
			Vector4 wy2yz2wz2;
			{
				var rot = rotation.Q;
				var xyz2 = rot + rot;
				xxyyzz2 = rot * xyz2;
				wx2xy2xz2 = xyz2 * new Vector4(rot.W, rot.X, rot.X,0f);
				wy2yz2wz2 = new Vector4(rot.W, rot.Y, rot.W, 0f);
				wy2yz2wz2 = wy2yz2wz2 * new Vector4(xyz2.Y, xyz2.Z, xyz2.Z, 0f);
			}
			Vector4 vr;
			{
				var t1 = new Vector4(1f, 0f, 0f, 0f);
				t1 = t1 + new Vector4(-xxyyzz2.Y, wx2xy2xz2.Y, wx2xy2xz2.Z, 0f);
				t1 = t1 + new Vector4(-xxyyzz2.Z, wy2yz2wz2.Z, -wy2yz2wz2.X, 0f);
				vr = t1 * new Vector4(vin.X);
				t1 = new Vector4(0f, 1f, 0f, 0f);
				t1 = t1 + new Vector4(wx2xy2xz2.Y, -xxyyzz2.X, wy2yz2wz2.Y, 0f);
				t1 = t1 + new Vector4(-wy2yz2wz2.Z, -xxyyzz2.Z, wx2xy2xz2.X, 0f);
				vr = vr + (t1 * new Vector4(vin.Y));
				t1 = new Vector4(0f, 0f, 1f, 0f);
				t1 = t1 + new Vector4(wx2xy2xz2.Z, wy2yz2wz2.Y, -xxyyzz2.X, 0f);
				t1 = t1 + new Vector4(wy2yz2wz2.X, -wx2xy2xz2.X, -xxyyzz2.Y, 0f);
				vr = vr + (t1 * new Vector4(vin.Z));

			}
			//return new Vector3(vr.X, vr.Y, vr.Z);
			r.X = vr.X;
			r.Y = vr.Y;
			r.Z = vr.Z;
		}


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
		// so :
		/// <summary>
		/// Transforms a vector by the given quaternion.
		/// </summary>
		/// <param name="v">The Vector3 to transform.</param>
		/// <param name="q">The Quaternion to apply.</param>
		/// <returns>The transformed Vector3.</returns>
		public static void Transform(ref Vector3 v,ref Quaternion q,out Vector3 r)
		{
			Vector3 u = new Vector3(q.Q.X, q.Q.Y, q.Q.Z);
			float s = q.Q.W;
			//r = 2.0f * Vector3.Dot(u, v) * u
			//	+ 2.0f * s * Vector3.Cross(u, v) 
			//	+ (s * s - Vector3.Dot(u, u)) * v;
			r = 2.0f * (Vector3.Dot(u, v) * u + s * Vector3.Cross(u, v))
				+ (s * s - Vector3.Dot(u, u)) * v;
		}

		/// <summary>
		/// Transforms an array of Vectors by the given quaternion.
		/// </summary>
		/// <param name="varray">The Vectors to transform.</param>
		/// <param name="q">The quaternion to apply.</param>
		/// <returns>The transformed Vectors</returns>
		public static void Transform(ref Vector3[] varray,ref Quaternion q,ref Vector3[] voutarray)
		{
			Vector3 u = new Vector3(q.Q.X, q.Q.Y, q.Q.Z);
			float s = q.Q.W;
			float u3D = s * s - Vector3.Dot(u, u);
			//Vector3[] ret = new Vector3[varray.Length];
			for (int i = 0; i < varray.Length; i++)
				voutarray[i] = (Vector3.Dot(u, varray[i]) * u + s * Vector3.Cross(u, varray[i])) * 2.0f
					+ varray[i] * u3D;
			//voutarray[i] = Vector3.Dot(u, varray[i]) * u * 2.0f
			//	+ 2.0f * s * Vector3.Cross(u, varray[i]) 
			//	+ (s * s - u3D) * varray[i];
			//voutarray = ret;
			//return ret;
		}


		public static Quaternion operator *(Quaternion a, Quaternion b)
		{
			Quaternion r;
			Multiply(ref a, ref b, out r);
			return r;
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
		public static void GetRelativeRotation(ref Quaternion start, ref Quaternion end,out Quaternion r)
		{
			Quaternion startInverse;
			Conjugate(ref start,out startInverse);
			Concatenate(ref startInverse,ref end,out r);
		}

		public static float Dot(ref Quaternion a,ref Quaternion b)
		{
			return Vector4.Dot(a.Q, b.Q);
		}
		/// <summary>
		/// Transforms the rotation into the local space of the target basis such that rotation = Quaternion.Concatenate(localRotation, targetBasis)
		/// </summary>
		/// <param name="rotation">Rotation in the original frame of reference.</param>
		/// <param name="targetBasis">Basis in the original frame of reference to transform the rotation into.</param>
		/// <param name="localRotation">Rotation in the local space of the target basis.</param>
		public static void GetLocalRotation(ref Quaternion rotation,ref Quaternion targetBasis,out Quaternion r)
		{
			Quaternion basisInverse;
			Conjugate(ref targetBasis,out basisInverse);
			Concatenate(ref rotation,ref basisInverse,out r);
		}

		/// <summary>
		/// Gets a string representation of the quaternion.
		/// </summary>
		/// <returns>String representing the quaternion.</returns>
		public override string ToString()
		{
			CultureInfo currentCulture = CultureInfo.CurrentCulture;
			return string.Format(currentCulture, "{{X:{0} Y:{1} Z:{2} W:{3}}}", new object[]
			{
			X.ToString(currentCulture),
			Y.ToString(currentCulture),
			Z.ToString(currentCulture),
			W.ToString(currentCulture)
			});
		}

		private static Quaternion _Identity = new Quaternion(0f, 0f, 0f, 1f);


	}
}
