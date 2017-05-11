using System;
using System.Numerics;
using System.Diagnostics;

namespace Tests
{

	public static class QuatPerfTest
	{
		public static void DoTest()
		{
			Quaternion[] quats = new Quaternion[100];
			Random random = new Random();
			Console.WriteLine("System.Numerics.Quaternion Performance Tests");
			Console.Write("Generating Quaternion array...");
			Quaternion rotation = Quaternion.CreateFromAxisAngle(Vector3.Normalize(new Vector3(1.0f, 2.0f, 3.0f)),
																MathHelper.ToRadians(30.0f));
			for (int i = 0; i < quats.Length; i++)
			{
				quats[i] = Quaternion.Normalize(new Quaternion((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()));
			}
			Console.WriteLine("done!");
			Vector3[] mesh = new Vector3[300];
			Console.Write("Generating vector3 array...");
			for (int i = 0; i < quats.Length; i++)
			{
				mesh[i] = Vector3.Normalize(new Vector3((float)(random.NextDouble() * 4.0d), (float)(random.NextDouble() * 4.0d), (float)(random.NextDouble() * 4.0d)));
			}
			Console.WriteLine("done!");
			float deltaTime = 5 / 20;
			GC.Collect();

			NormalizePerf();
			ConcatenatePerf();
			MultPerf(ref quats, ref rotation);
			MultPref2(ref quats, ref rotation);
			SlerpPerf(ref quats[random.Next(100)], ref quats[random.Next(100)], deltaTime);
			TransformPerf(ref mesh, ref quats[random.Next(100)]);
			Console.WriteLine("System.Numerics.Quaternion Perfomance test complete.\n");

		}
		public static void MultPerf(ref Quaternion[] qarray, ref Quaternion addRot)
		{
			Quaternion[] qout = new Quaternion[qarray.Length];
			Console.Write("System.Numerics.Quaternion.op_multiply Iterations: {0} : ", 1000000 * qarray.Length);
			var timestart = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
			for (int t = 0; t < 1000000; t++)
			{
				for (int i = 0; i < qarray.Length; i++)
				{
					qout[i] = qarray[i] * addRot;
				}
			}
			var timerend = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
			Console.WriteLine(timerend - timestart);
		}
		public static void MultPref2(ref Quaternion[] qarray, ref Quaternion addRot)
		{
			Console.Write("System.Numerics.Quaternion.Multiply    Iterations: {0} : ", 1000000 * qarray.Length);
			Quaternion[] qout = new Quaternion[qarray.Length];
			var timestart = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
			for (int t = 0; t < 1000000; t++)
			{
				for (int i = 0; i < qarray.Length; i++)
				{
					qout[i] = Quaternion.Multiply(qarray[i], addRot);
				}
			}
			var timerend = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
			Console.WriteLine(timerend - timestart);
		}
		public static void SlerpPerf(ref Quaternion qstart, ref Quaternion qend, float deltaamount)
		{
			Quaternion qout;
			Console.Write("System.Numerics.Quaternion.Slerp       Iterations: 100000000 : ");
			var timestart = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
			for (int t = 0; t < 100000000; t++)
			{
				qout = Quaternion.Slerp(qstart, qend, deltaamount);
			}
			var timerend = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
			Console.WriteLine(timerend - timestart);

		}
		public static void TransformPerf(ref Vector3[] mesharray, ref Quaternion q)
		{
			Vector3[] meshout = new Vector3[mesharray.Length];
			Console.WriteLine("System.Numerics.Quaternion.Transform   Iterations: 300000000");
			var timestart = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
			for(int i=0;i< 1000000;i++)
				for (int t = 0; t < 300; t++)
					meshout[t] = q.Transform(mesharray[t]);
			var timerend = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
			Console.WriteLine("                   Transform                            tics : {0}", timerend - timestart);

			timestart = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
			for (int i = 0; i < 1000000; i++)
				for (int t = 0; t < 300; t++)
					meshout[t] = q.TransformLong(mesharray[t]);
			timerend = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
			Console.WriteLine("                   TransformLong                        tics : {0}", timerend - timestart);

			timestart = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
			for (int i = 0; i < 1000000; i++)
				for (int t = 0; t < 300; t++)
					meshout[t] = Vector3.Transform(mesharray[t], q);
			timerend = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
			Console.WriteLine("                   Vector3.Transform                    tics : {0}", timerend - timestart);

			timestart = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
			for (int t = 0; t < 1000000; t++)
				meshout = q.Transform(mesharray);
			timerend = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
			Console.WriteLine("                   Transform(array)-Elements:{0}        tics : {1}", mesharray.Length, timerend - timestart);

		}
		public static void NormalizePerf()
		{
			Quaternion a = new Quaternion(1.0f, 2.0f, 3.0f, 4.0f);
			Quaternion output;
			Console.Write("System.Numerics.Quaternion.Normalize   Iterations: 100000000 : ");
			var timestart = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
			for (int i = 0; i < 100000000; i++)
				output = Quaternion.Normalize(a);
			var timeend = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
			Console.WriteLine(timeend - timestart);
		}
		public static void ConcatenatePerf()
		{
			Console.Write("System.Numerics.Quaternion.Concatenate Iterations: 100000000 : ");
			Quaternion b = Quaternion.Normalize(new Quaternion(1.0f, 2.0f, 3.0f, 4.0f));
			Quaternion a = Quaternion.Normalize(new Quaternion(5.0f, 6.0f, 7.0f, 8.0f));
			Quaternion output;
			var timestart = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
			for (int i = 0; i < 100000000; i++)
				output = Quaternion.Concatenate(a, b);
			var timeend = Stopwatch.GetTimestamp() / (double)Stopwatch.Frequency;
			Console.WriteLine(timeend - timestart);
		}
	}
}

