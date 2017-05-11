using System;
using System.Globalization;
using System.Numerics;
using System.Runtime.CompilerServices;
namespace Vectorized
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

		public Quaternion(Vector3 v, float w):this(v.X,v.Y,v.Z,w)
		{}

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

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Quaternion Add(Quaternion left, Quaternion right)
		{
			return new Quaternion(left.Q + right.Q);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Quaternion Subtract(Quaternion left, Quaternion right)
		{
			return new Quaternion(left.Q - right.Q);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Quaternion operator +(Quaternion left, Quaternion right)
		{
			return Add(left, right);
		}
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Quaternion operator -(Quaternion left, Quaternion right)
		{
			return Subtract(left, right);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Quaternion Multiply(Quaternion left, Quaternion right)
		{
			Vector3 a = new Vector3(left.X, left.Y, left.Z);
			Vector3 b = new Vector3(right.X, right.Y, right.Z);
			return new Quaternion(b * left.W + a * right.W + Vector3.Cross(a, b), left.W * right.W - Vector3.Dot(a, b));
		}

		/// <summary>
		/// Scales a quaternion.
		/// </summary>
		/// <param name="q">Quaternion to multiply.</param>
		/// <param name="scale">Amount to multiply each component of the quaternion by.</param>
		/// <param name="result">Scaled quaternion.</param>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Quaternion Scale(Quaternion q, float scale)
		{
			Quaternion r;
			r.Q = q.Q * scale;
			return r;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Quaternion operator *(Quaternion a, float b)
		{
			return Scale(a, b);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Quaternion Divide(Quaternion value1, Quaternion value2)
		{
			var v1 = value1.Q;
			Vector4 v3;
			{
				var v2 = value2.Q;
				float num2 = 1f / Vector4.Dot(v2, v2);
				v3 = Conjugate(value2).Q * num2;
			}
			Quaternion r;
			{
				var nv3 = -v3;
				var v = new Vector4(v3.W, nv3.Z, v3.Y, nv3.X) * v1.X;
				v = v + (new Vector4(v3.Z, v3.W, nv3.X, nv3.Y) * v1.Y);
				v = v + (new Vector4(nv3.Y, v3.X, v3.W, nv3.Z) * v1.Z);
				v = v + (v3 * v1.W);
				r.Q = v;
			}

			return r;
		}
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Quaternion operator /(Quaternion l, Quaternion r)
		{
			return Divide(l, r);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Quaternion Concatenate(Quaternion left, Quaternion right)
		{
			Vector3 a = new Vector3(right.X, right.Y, right.Z);
			Vector3 b = new Vector3(left.X, left.Y, left.Z);
			return new Quaternion(b * left.W + a * right.W + Vector3.Cross(a, b), left.W * right.W - Vector3.Dot(a, b));
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Quaternion Normalize(Quaternion quaternion)
		{
			Quaternion r;
			var q = quaternion.Q;
			//var inverse = Vector<float>.One / Vector.SquareRoot(new Vector<float>(Vector.Dot(q,q))) ;
			r.Q = q / (float)Math.Sqrt( Vector4.Dot(q, q));
			return r;
		}
		public float LengthSquared()
		{
			return Vector4.Dot(Q, Q);
		}

		public float Length()
		{
			var d = new Vector4(Vector4.Dot(Q, Q));
			return Vector4.SquareRoot(d).X;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Quaternion Slerp(Quaternion start, Quaternion end, float interpolationAmount)
		{

			// Original SLERP
			// q0 and qn are unit quaternions
			// cos(alpha) = Dot(q0,qn) 
			// aplha is the angle between q0 and qn
			// qt = 1/sin(alpha)*(sin(alpha-t*alpha)*q0 + sin(t*alpha)*qn)
			// qt = (p*sin((1-t)*alpha) + q*sin(t*aplha))/sin(alpha)
			/*
				var fastSlerpScratchQuaternion = new Quaternion();
				var opmu = 1.90110745351730037;
				var u =  FeatureDetection.supportsTypedArrays() ? new Float32Array(8) : [];
				var v =  FeatureDetection.supportsTypedArrays() ? new Float32Array(8) : [];
				var bT = FeatureDetection.supportsTypedArrays() ? new Float32Array(8) : [];
				var bD = FeatureDetection.supportsTypedArrays() ? new Float32Array(8) : [];
				var t = interpolationAmount;
				// This can be done in a few vectors(3)?
				// Static construction method!
				for (var i = 0; i < 7; ++i) 
				{
					var s = i + 1.0;
					var t = 2.0 * s + 1.0;
					u[i] = 1.0 / (s * t);
					v[i] = s / t;
				}

				u[7] = opmu / (8.0 * 17.0);
				v[7] = opmu * 8.0 / 17.0;
				---------------------------------
				var x = Quaternion.dot(start, end);

				var sign;
				if (x >= 0) 
				{
					sign = 1.0;
				}
				else 
				{
					sign = -1.0;
					x = -x;
				}

				var xm1 = x - 1.0;
				var d = 1.0 - t;
				var sqrT = t * t;
				var sqrD = d * d;

				for (var i = 7; i >= 0; --i) 
				{
					bT[i] = (u[i] * sqrT - v[i]) * xm1;
					bD[i] = (u[i] * sqrD - v[i]) * xm1;
				}

				var cT = sign * t * (
					1.0 + bT[0] * (1.0 + bT[1] * (1.0 + bT[2] * (1.0 + bT[3] * (
					1.0 + bT[4] * (1.0 + bT[5] * (1.0 + bT[6] * (1.0 + bT[7]))))))));
				var cD = d * (
					1.0 + bD[0] * (1.0 + bD[1] * (1.0 + bD[2] * (1.0 + bD[3] * (
					1.0 + bD[4] * (1.0 + bD[5] * (1.0 + bD[6] * (1.0 + bD[7]))))))));

				var temp = Quaternion.multiplyByScalar(start, cD, fastSlerpScratchQuaternion);
				Quaternion.multiplyByScalar(end, cT, result);
				return Quaternion.add(temp, result, result);

			*/
			var e = end.Q;
			var s = start.Q;
			Quaternion r;
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
			//var va = (float)aFraction;
			//var vb = (float)bFraction;
			r.Q = ((s * (float)aFraction) + (e * (float)bFraction));
			return r;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Quaternion Conjugate(Quaternion quaternion)
		{
			Quaternion q;
			q.Q.X = -quaternion.Q.X;
			q.Q.Y = -quaternion.Q.Y;
			q.Q.Z = -quaternion.Q.Z;
			q.Q.W = quaternion.Q.W;
			return q;
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Quaternion Inverse(Quaternion quaternion)
		{
			var q = quaternion.Q;
			return Scale(Conjugate(quaternion), 1f / Vector4.Dot(q,q));
		}
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static bool operator ==(Quaternion a, Quaternion b)
		{
			return (a.Q == b.Q);
		}
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static bool operator !=(Quaternion a, Quaternion b)
		{
			return !(a == b);
		}

		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Quaternion Negate(Quaternion q)
		{
			Quaternion r;
			r.Q = -q.Q;
			return r;
		}
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
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
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Vector3 TransformLong(Vector3 v, Quaternion rotation)
		{
			Vector4 vin = new Vector4(v.X, v.Y, v.Z, 0f);
			Vector4 xxyyzz2;
			Vector4 wx2xy2xz2;
			Vector4 wy2yz2wz2;
			{
				var rot = rotation.Q;
				var xyz2 = rot + rot;
				xxyyzz2 = rot * xyz2;
				wx2xy2xz2 = xyz2 * new Vector4( rot.W, rot.X, rot.X, 0f );
				wy2yz2wz2 = new Vector4(rot.W, rot.Y, rot.W, 0f);
				wy2yz2wz2 = wy2yz2wz2 * new Vector4(xyz2.Y, xyz2.Z, xyz2.Z, 0f);
			}
			Vector4 vr;
			{
				var t1 = new Vector4(1f,0f,0f,0f);
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
			return new Vector3(vr.X,vr.Y,vr.Z);
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
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Vector3 Transform(Vector3 v, Quaternion q)
		{
			Vector3 u = new Vector3(q.Q.X, q.Q.Y, q.Q.Z);
			float s = q.Q.W;
			var r = (Vector3.Dot(u, v) * u + s * Vector3.Cross(u, v)) * 2.0f
				+ v * (s * s - Vector3.Dot(u, u));
			//var r = 2.0f * u * Vector3.Dot(u, v)
			//	+ 2.0f * s * Vector3.Cross(u, v) 
			//	+ (s * s - Vector3.Dot(u, u)) * v;
			return r;
		}

		/// <summary>
		/// Transforms an array of Vectors by the given quaternion.
		/// </summary>
		/// <param name="varray">The Vectors to transform.</param>
		/// <param name="q">The quaternion to apply.</param>
		/// <returns>The transformed Vectors</returns>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Vector3 [] Transform(Vector3 [] varray, Quaternion q)
		{
			Vector3 u = new Vector3(q.Q.X, q.Q.Y, q.Q.Z);
			float s = q.Q.W;
			float v3D = s * s - Vector3.Dot(u, u);
			Vector3[] ret = new Vector3[varray.Length];
			for (int i = 0; i < varray.Length; i++)
				ret[i] = (Vector3.Dot(u, varray[i]) * u + s * Vector3.Cross(u, varray[i])) * 2.0f
					+ varray[i] * v3D;
			//ret[i] = u * 2.0f * Vector3.Dot(u, varray[i])
			//		+ 2.0f * s * Vector3.Cross(u, varray[i])
			//		+ (s * s - v3D) * varray[i];
			return ret;
		}


		[MethodImpl(MethodImplOptions.AggressiveInlining)]
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
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Quaternion GetRelativeRotation(ref Quaternion start, ref Quaternion end)
		{
			Quaternion startInverse = Conjugate(start);
			return Concatenate(startInverse, end);
		}

		public static float Dot(Quaternion a,Quaternion b)
		{
			return Vector4.Dot(a.Q, b.Q);
		}
		/// <summary>
		/// Transforms the rotation into the local space of the target basis such that rotation = Quaternion.Concatenate(localRotation, targetBasis)
		/// </summary>
		/// <param name="rotation">Rotation in the original frame of reference.</param>
		/// <param name="targetBasis">Basis in the original frame of reference to transform the rotation into.</param>
		/// <param name="localRotation">Rotation in the local space of the target basis.</param>
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		public static Quaternion GetLocalRotation(Quaternion rotation, Quaternion targetBasis)
		{
			Quaternion basisInverse = Conjugate(targetBasis);
			return Concatenate(rotation, basisInverse);
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
