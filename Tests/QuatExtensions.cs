using System.Numerics;
namespace Tests
{
    static class QuatExtensions
    {
		public static Vector3 Forward(this Quaternion q)
		{
			return new Vector3( -2 * (q.X * q.Z + q.W * q.Y),
								-2 * (q.Y * q.Z - q.W * q.X),
								-1 + 2 * (q.X * q.X + q.Y * q.Y));
		}

		public static Vector3 Up(this Quaternion q)
		{
			return new Vector3( 2 * (q.X * q.Y - q.W * q.Z),
								1 - 2 * (q.X * q.X + q.Z * q.Z),
								2 * (q.Y * q.Z + q.W * q.X));
		}

		public static Vector3 Right(this Quaternion q)
		{
			return new Vector3( 1 - 2 * (q.Y * q.Y + q.Z * q.Z),
								2 * (q.X * q.Y + q.W * q.Z),
								2 * (q.X * q.Z - q.W * q.Y));
		}

		public static Vector3 Left(this Quaternion q)
		{
			return new Vector3( -1 + 2 * (q.Y * q.Y + q.Z * q.Z),
								-2 * (q.X * q.Y + q.W * q.Z),
								-2 * (q.X * q.Z - q.W * q.Y));
		}
		public static Vector3 Down(this Quaternion q)
		{
			return new Vector3( -2 * (q.X * q.Y - q.W * q.Z),
								-1 + 2 * (q.X * q.X + q.Z * q.Z),
								-2 * (q.Y * q.Z + q.W * q.X));
		}
		public static Vector3 Back(this Quaternion q)
		{
			return new Vector3( 2 * (q.X * q.Z + q.W * q.Y),
								2 * (q.Y * q.Z - q.W * q.X),
								1 - 2 * (q.X * q.X + q.Y * q.Y));
		}


		public static Vector3 TransformLong(this Quaternion rotation, Vector3 v)
		{
			Vector3 result;
			//This operation is an optimized-down version of v' = q * v * q^-1.
			//The expanded form would be to treat v as an 'axis only' quaternion
			//and perform standard quaternion multiplication.  Assuming q is normalized,
			//q^-1 can be replaced by a conjugation.
			float x2 = rotation.X + rotation.X;
			float y2 = rotation.Y + rotation.Y;
			float z2 = rotation.Z + rotation.Z;
			float xx2 = rotation.X * x2;
			float xy2 = rotation.X * y2;
			float xz2 = rotation.X * z2;
			float yy2 = rotation.Y * y2;
			float yz2 = rotation.Y * z2;
			float zz2 = rotation.Z * z2;
			float wx2 = rotation.W * x2;
			float wy2 = rotation.W * y2;
			float wz2 = rotation.W * z2;
			//Defer the component setting since they're used in computation.
			float transformedX = v.X * (1f - yy2 - zz2) + v.Y * (xy2 - wz2) + v.Z * (xz2 + wy2);
			float transformedY = v.X * (xy2 + wz2) + v.Y * (1f - xx2 - zz2) + v.Z * (yz2 - wx2);
			float transformedZ = v.X * (xz2 - wy2) + v.Y * (yz2 + wx2) + v.Z * (1f - xx2 - yy2);
			result.X = transformedX;
			result.Y = transformedY;
			result.Z = transformedZ;
			return result;
		}
		/// <summary>
		/// Transforms an array of Vectors by the given quaternion.
		/// </summary>
		/// <param name="varray">The Vectors to transform.</param>
		/// <param name="q">The quaternion to apply.</param>
		/// <returns>The transformed Vectors</returns>
		public static Vector3[] Transform(this Quaternion q, Vector3[] varray )
		{
			Vector3 u = new Vector3(q.X, q.Y, q.Z);
			float s = q.W;
			//float v3D = Vector3.Dot(u, u);
			float v3Dc = s * s - Vector3.Dot(u,u);
			Vector3[] ret = new Vector3[varray.Length];
			for (int i = 0; i < varray.Length; i++)
				//ret[i] = 2.0f * (Vector3.Dot(u, varray[i]) * u + s * Vector3.Cross(u, varray[i]))
				//	+ (s * s - v3D) * varray[i];
				//ret[i] = 2.0f * Vector3.Dot(u, varray[i]) * u
				//	+ 2.0f * s * Vector3.Cross(u, varray[i]) 
				//	+ (s * s - v3D) * varray[i];
				// Where u is the vector part of the quaternion and 
				// s is the scalar part. It runs about 15% faster than the formula that you posted here.
				ret[i] = (u * Vector3.Dot(u, varray[i]) + Vector3.Cross(u, varray[i]) * (s)) * 2.0f
						 + varray[i] * v3Dc;
			return ret;
		}
		/// <summary>
		/// Transforms a vector by the given quaternion.
		/// </summary>
		/// <param name="v">The Vector3 to transform.</param>
		/// <param name="q">The Quaternion to apply.</param>
		/// <returns>The transformed Vector3.</returns>
		public static Vector3 Transform(this Quaternion q, Vector3 v )
		{
			Vector3 u = new Vector3(q.X, q.Y, q.Z);
			float s = q.W;
			float v3Dc = s * s - Vector3.Dot(u, u);
			var r = (u * Vector3.Dot(u, v) + Vector3.Cross(u, v) * (s)) * 2.0f + v * v3Dc;
			return r;
		}


	}
}
