using System;
using System.Collections.Generic;
using System.Text;
using System.Numerics;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Globalization;

namespace Vectorized
{
	/// <summary>
	/// Tests the results of the Quaternion fuctions.
	/// </summary>
	public class QuaternionTests
	{
		// A test for Dot (Quaternion, Quaternion)
		public void QuaternionDotTest()
		{
			Quaternion a = new Quaternion(1.0f, 2.0f, 3.0f, 4.0f);
			Quaternion b = new Quaternion(5.0f, 6.0f, 7.0f, 8.0f);

			float expected = 70.0f;
			float actual;

			actual = Quaternion.Dot(a, b);
			Debug.Assert(MathHelper.Equal(expected, actual), "Quaternion.Dot did not return the expected value."); 
		}

		// A test for Length ()
		public void QuaternionLengthTest()
		{
			Vector3 v = new Vector3(1.0f, 2.0f, 3.0f);

			float w = 4.0f;

			Quaternion target = new Quaternion(v, w);

			float expected = 5.477226f;
			float actual;

			actual = target.Length();

			Debug.Assert(MathHelper.Equal(expected, actual), "Quaternion.Length did not return the expected value."); 
		}

		// A test for LengthSquared ()
		public void QuaternionLengthSquaredTest()
		{
			Vector3 v = new Vector3(1.0f, 2.0f, 3.0f);
			float w = 4.0f;

			Quaternion target = new Quaternion(v, w);

			float expected = 30.0f;
			float actual;

			actual = target.LengthSquared();

			Debug.Assert(MathHelper.Equal(expected, actual), "Quaternion.LengthSquared did not return the expected value."); 
		}
		public void QuaternionConjugateTest1()
		{
			Quaternion a = new Quaternion(1, 2, 3, 4);

			Quaternion expected = new Quaternion(-1, -2, -3, 4);
			Quaternion actual;

			actual = Quaternion.Conjugate(a);
			Debug.Assert(MathHelper.Equal(expected, actual), "Quaternion.Conjugate did not return the expected value."); 
		}

		// A test for Normalize (Quaternion)
		public void QuaternionNormalizeTest()
		{
			Quaternion a = new Quaternion(1.0f, 2.0f, 3.0f, 4.0f);

			Quaternion expected = new Quaternion(0.182574168f, 0.365148336f, 0.5477225f, 0.7302967f);
			Quaternion actual;

			actual = Quaternion.Normalize(a);
			Debug.Assert(MathHelper.Equal(expected, actual), "Quaternion.Normalize did not return the expected value."); 
		}

		// A test for Normalize (Quaternion)
		// Normalize zero length quaternion
		public void QuaternionNormalizeTest1()
		{
			Quaternion a = new Quaternion(0.0f, 0.0f, -0.0f, 0.0f);

			Quaternion actual = Quaternion.Normalize(a);
			Debug.Assert(float.IsNaN(actual.X) && float.IsNaN(actual.Y) && float.IsNaN(actual.Z) && float.IsNaN(actual.W)
			, "Quaternion.Normalize did not return the expected value."); 
		}

		// A test for Concatenate(Quaternion, Quaternion)

		public void QuaternionConcatenateTest1()
		{
			Quaternion b = new Quaternion(1.0f, 2.0f, 3.0f, 4.0f);
			Quaternion a = new Quaternion(5.0f, 6.0f, 7.0f, 8.0f);

			Quaternion expected = new Quaternion(24.0f, 48.0f, 48.0f, -6.0f);
			Quaternion actual;

			actual = Quaternion.Concatenate(a, b);
			Debug.Assert(MathHelper.Equal(expected, actual), "Quaternion.Concatenate did not return the expected value."); 
		}

		// A test for operator - (Quaternion, Quaternion)
		public void QuaternionSubtractionTest()
		{
			Quaternion a = new Quaternion(1.0f, 6.0f, 7.0f, 4.0f);
			Quaternion b = new Quaternion(5.0f, 2.0f, 3.0f, 8.0f);

			Quaternion expected = new Quaternion(-4.0f, 4.0f, 4.0f, -4.0f);
			Quaternion actual;

			actual = a - b;

			Debug.Assert(MathHelper.Equal(expected, actual), "Quaternion.operator - did not return the expected value."); 
		}

		// A test for operator * (Quaternion, float)
		public void QuaternionMultiplyTest()
		{
			Quaternion a = new Quaternion(1.0f, 2.0f, 3.0f, 4.0f);
			float factor = 0.5f;

			Quaternion expected = new Quaternion(0.5f, 1.0f, 1.5f, 2.0f);
			Quaternion actual;

			actual = a * factor;

			Debug.Assert(MathHelper.Equal(expected , actual), "Quaternion.operator * did not return the expected value."); 
		}

		// A test for operator * (Quaternion, Quaternion)
		public void QuaternionMultiplyTest1()
		{
			Quaternion a = new Quaternion(1.0f, 2.0f, 3.0f, 4.0f);
			Quaternion b = new Quaternion(5.0f, 6.0f, 7.0f, 8.0f);

			Quaternion expected = new Quaternion(24.0f, 48.0f, 48.0f, -6.0f);
			Quaternion actual;

			actual = a * b;

			Debug.Assert(MathHelper.Equal(expected , actual), "Quaternion.operator * did not return the expected value."); 
		}

		// A test for operator / (Quaternion, Quaternion)
		public void QuaternionDivisionTest1()
		{
			Quaternion a = new Quaternion(1.0f, 2.0f, 3.0f, 4.0f);
			Quaternion b = new Quaternion(5.0f, 6.0f, 7.0f, 8.0f);

			Quaternion expected = new Quaternion(-0.045977015f, -0.09195402f, -7.450581E-9f, 0.402298868f);
			Quaternion actual;

			actual = a / b;

			Debug.Assert(MathHelper.Equal(expected , actual), "Quaternion.operator / did not return the expected value."); 
		}

		// A test for operator + (Quaternion, Quaternion)
		public void QuaternionAdditionTest()
		{
			Quaternion a = new Quaternion(1.0f, 2.0f, 3.0f, 4.0f);
			Quaternion b = new Quaternion(5.0f, 6.0f, 7.0f, 8.0f);

			Quaternion expected = new Quaternion(6.0f, 8.0f, 10.0f, 12.0f);
			Quaternion actual;

			actual = a + b;

			Debug.Assert((expected == actual), "Quaternion.operator + did not return the expected value."); 
		}

		// A test for CreateFromAxisAngle (Vector3f, float)

		public void QuaternionCreateFromAxisAngleTest()
		{
			Vector3 axis = Vector3.Normalize(new Vector3(1.0f, 2.0f, 3.0f));
			float angle = MathHelper.ToRadians(30.0f);
			

			Quaternion expected = new Quaternion(0.0691723f, 0.1383446f, 0.207516879f, 0.9659258f);
			Quaternion actual = Quaternion.CreateFromAxisAngle(axis, angle);
			Debug.Assert(MathHelper.Equal(expected, actual), "Quaternion.CreateFromAxisAngle did not return the expected value.");
		}

		// A test for CreateFromAxisAngle (Vector3f, float)
		// CreateFromAxisAngle of zero vector
		public void QuaternionCreateFromAxisAngleTest1()
		{
			Vector3 axis = new Vector3();
			float angle = MathHelper.ToRadians(-30.0f);

			float cos = (float)System.Math.Cos(angle / 2.0f);
			Quaternion actual = Quaternion.CreateFromAxisAngle(axis, angle);

			Debug.Assert(actual.X == 0.0f && actual.Y == 0.0f && actual.Z == 0.0f && cos== actual.W
				, "Quaternion.CreateFromAxisAngle did not return the expected value.");
		}

		// A test for CreateFromAxisAngle (Vector3f, float)
		// CreateFromAxisAngle of angle = 30 && 750
		public void QuaternionCreateFromAxisAngleTest2()
		{
			Vector3 axis = new Vector3(1f, 0f, 0f);
			Debug.Assert(axis.X == 1F && axis.Y == 0f && axis.Z == 0f, "Vector3 Creation failed!");
			float angle1 = MathHelper.ToRadians(30.0f);
			float angle2 = MathHelper.ToRadians(750.0f);

			Quaternion actual1 = Quaternion.CreateFromAxisAngle(axis, angle1);
			Quaternion actual2 = Quaternion.CreateFromAxisAngle(axis, angle2);
			Debug.Assert(MathHelper.Equal(actual1,  actual2), "Quaternion.CreateFromAxisAngle did not return the expected value.");
		}

		// A test for CreateFromAxisAngle (Vector3f, float)
		// CreateFromAxisAngle of angle = 30 && 390
		public void QuaternionCreateFromAxisAngleTest3()
		{
			Vector3 axis = new Vector3(1f, 0f, 0f);
			float angle1 = MathHelper.ToRadians(30.0f);
			float angle2 = MathHelper.ToRadians(390.0f);

			Quaternion actual1 = Quaternion.CreateFromAxisAngle(axis, angle1);
			Quaternion actual2 = Quaternion.CreateFromAxisAngle(axis, angle2);
			actual1.Q.X = -actual1.X;
			actual1.Q.W = -actual1.W;

			Debug.Assert(MathHelper.Equal(actual1, actual2), "Quaternion.CreateFromAxisAngle did not return the expected value.");
		}

		public void QuaternionSlerpTest()
		{
			Vector3 axis = Vector3.Normalize(new Vector3(1.0f, 2.0f, 3.0f));
			Quaternion a = Quaternion.CreateFromAxisAngle(axis, MathHelper.ToRadians(10.0f));
			Quaternion b = Quaternion.CreateFromAxisAngle(axis, MathHelper.ToRadians(30.0f));

			float t = 0.5f;

			Quaternion expected = Quaternion.CreateFromAxisAngle(axis, MathHelper.ToRadians(20.0f));
			Quaternion actual;

			actual = Quaternion.Slerp(a, b, t);
			Debug.Assert(MathHelper.Equal(expected , actual), "Quaternion.Slerp did not return the expected value.");

			// Case a and b are same.
			expected = a;
			actual = Quaternion.Slerp(a, a, t);
			Debug.Assert(MathHelper.Equal(expected, actual), "Quaternion.Slerp did not return the expected value.");
		}

		// A test for Slerp (Quaternion, Quaternion, float)
		// Slerp test where t = 0
		public void QuaternionSlerpTest1()
		{
			Vector3 axis = Vector3.Normalize(new Vector3(1.0f, 2.0f, 3.0f));
			Quaternion a = Quaternion.CreateFromAxisAngle(axis, MathHelper.ToRadians(10.0f));
			Quaternion b = Quaternion.CreateFromAxisAngle(axis, MathHelper.ToRadians(30.0f));

			float t = 0.0f;

			Quaternion expected = new Quaternion(a.X, a.Y, a.Z, a.W);
			Quaternion actual = Quaternion.Slerp(a, b, t);
			Debug.Assert(MathHelper.Equal(expected, actual), "Quaternion.Slerp did not return the expected value.");
		}

		// A test for Slerp (Quaternion, Quaternion, float)
		// Slerp test where t = 1
		public void QuaternionSlerpTest2()
		{
			Vector3 axis = Vector3.Normalize(new Vector3(1.0f, 2.0f, 3.0f));
			Quaternion a = Quaternion.CreateFromAxisAngle(axis, MathHelper.ToRadians(10.0f));
			Quaternion b = Quaternion.CreateFromAxisAngle(axis, MathHelper.ToRadians(30.0f));

			float t = 1.0f;

			Quaternion expected = new Quaternion(b.X, b.Y, b.Z, b.W);
			Quaternion actual = Quaternion.Slerp(a, b, t);
			Debug.Assert(MathHelper.Equal(expected, actual), "Quaternion.Slerp did not return the expected value.");
		}

		// A test for Slerp (Quaternion, Quaternion, float)
		// Slerp test where dot product is < 0
		public void QuaternionSlerpTest3()
		{
			Vector3 axis = Vector3.Normalize(new Vector3(1.0f, 2.0f, 3.0f));
			Quaternion a = Quaternion.CreateFromAxisAngle(axis, MathHelper.ToRadians(10.0f));
			Quaternion b = -a;

			float t = 1.0f;

			Quaternion expected = a;
			Quaternion actual = Quaternion.Slerp(a, b, t);
			// Note that in quaternion world, Q == -Q. In the case of quaternions dot product is zero, 
			// one of the quaternion will be flipped to compute the shortest distance. When t = 1, we
			// expect the result to be the same as quaternion b but flipped.
			Debug.Assert(MathHelper.Equal(expected, actual), "Quaternion.Slerp did not return the expected value.");
		}

		// A test for Slerp (Quaternion, Quaternion, float)
		// Slerp test where the quaternion is flipped
		public void QuaternionSlerpTest4()
		{
			Vector3 axis = Vector3.Normalize(new Vector3(1.0f, 2.0f, 3.0f));
			Quaternion a = Quaternion.CreateFromAxisAngle(axis, MathHelper.ToRadians(10.0f));
			Quaternion b = -Quaternion.CreateFromAxisAngle(axis, MathHelper.ToRadians(30.0f));

			float t = 0.0f;

			Quaternion expected = new Quaternion(a.X, a.Y, a.Z, a.W);
			Quaternion actual = Quaternion.Slerp(a, b, t);
			Debug.Assert(MathHelper.Equal(expected, actual), "Quaternion.Slerp did not return the expected value.");
		}
		// A test for operator - (Quaternion)
		public void QuaternionUnaryNegationTest()
		{
			Quaternion a = new Quaternion(1.0f, 2.0f, 3.0f, 4.0f);

			Quaternion expected = new Quaternion(-1.0f, -2.0f, -3.0f, -4.0f);
			Quaternion actual;

			actual = -a;

			Debug.Assert(MathHelper.Equal(expected, actual), "Quaternion.operator - did not return the expected value.");
		}

		// A test for Inverse (Quaternion)
		public void QuaternionInverseTest()
		{
			Quaternion a = new Quaternion(5.0f, 6.0f, 7.0f, 8.0f);

			Quaternion expected = new Quaternion(-0.0287356321f, -0.03448276f, -0.0402298868f, 0.04597701f);
			Quaternion actual;

			actual = Quaternion.Inverse(a);
			Debug.Assert(MathHelper.Equal(expected, actual));
		}

		// A test for Inverse (Quaternion)
		// Invert zero length quaternion
		public void QuaternionInverseTest1()
		{
			Quaternion a = new Quaternion();
			Quaternion actual = Quaternion.Inverse(a);

			Debug.Assert(float.IsNaN(actual.X) && float.IsNaN(actual.Y) && float.IsNaN(actual.Z) && float.IsNaN(actual.W)
				);
		}

		// A test for ToString ()
		public void QuaternionToStringTest()
		{
			Quaternion target = new Quaternion(-1.0f, 2.2f, 3.3f, -4.4f);

			string expected = string.Format(CultureInfo.CurrentCulture
				, "{{X:{0} Y:{1} Z:{2} W:{3}}}"
				, -1.0f, 2.2f, 3.3f, -4.4f);

			string actual = target.ToString();
			Debug.Assert(expected == actual);
		}
		// A test for GetHashCode ()
		public void QuaternionGetHashCodeTest()
		{
			Quaternion a = new Quaternion(1.0f, 2.0f, 3.0f, 4.0f);

			int expected = unchecked(a.X.GetHashCode() + a.Y.GetHashCode() + a.Z.GetHashCode() + a.W.GetHashCode());
			int actual = a.GetHashCode();
			Debug.Assert(expected == actual);
		}
		// Defines the Quaternion as a std Vector4
		public Vector3 QuatStdTransform(Vector3 v, Quaternion rotation)
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
		public void QuaternionFastTransformTest()
		{
			Vector3 v = Vector3.Normalize(new Vector3(1f, 2f, 1.5f));
			Vector3 axis = Vector3.Normalize(new Vector3(1.0f, 2.0f, 3.0f));
			Quaternion a = Quaternion.CreateFromAxisAngle(axis, MathHelper.ToRadians(10.0f));
			Vector3 expected = QuatStdTransform(v, a);
			Vector3 actual = Quaternion.Transform(v, a);
			Debug.Assert(MathHelper.Equal(expected, actual), "Quaternion FastTransform test failed");

		}
		public void QuaternionTransformTest()
		{
			Vector3 v = Vector3.Normalize(new Vector3(1f, 2f, 1.5f));
			Vector3 axis = Vector3.Normalize(new Vector3(1.0f, 2.0f, 3.0f));
			Quaternion a = Quaternion.CreateFromAxisAngle(axis, MathHelper.ToRadians(10.0f));
			Vector3 expected = QuatStdTransform(v, a);
			Vector3 actual = Quaternion.Transform(v,a);
			Debug.Assert(MathHelper.Equal(expected, actual), "Quaternion Transform test failed");

		}

		public void DoTests()
		{
			QuaternionCreateFromAxisAngleTest();
			QuaternionCreateFromAxisAngleTest1();
			QuaternionCreateFromAxisAngleTest2();
			QuaternionCreateFromAxisAngleTest3();
			QuaternionAdditionTest();
			QuaternionConcatenateTest1();
			QuaternionConjugateTest1();
			QuaternionDivisionTest1();
			QuaternionDotTest();
			QuaternionGetHashCodeTest();
			QuaternionInverseTest();
			QuaternionInverseTest1();
			QuaternionLengthSquaredTest();
			QuaternionLengthTest();
			QuaternionMultiplyTest();
			QuaternionMultiplyTest1();
			QuaternionNormalizeTest();
			QuaternionNormalizeTest1();
			QuaternionSlerpTest();
			QuaternionSlerpTest1();
			QuaternionSlerpTest2();
			QuaternionSlerpTest3();
			QuaternionSlerpTest4();
			QuaternionSubtractionTest();
			QuaternionToStringTest();
			QuaternionUnaryNegationTest();
			QuaternionFastTransformTest();
			QuaternionTransformTest();
			
		}
	}
}