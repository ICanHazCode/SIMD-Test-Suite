using System;

namespace Tests
{
    class Program
    {
        static void Main(string[] args)
        {
			Vectorized.QuaternionTests t = new Vectorized.QuaternionTests();
			t.DoTests();
			Console.WriteLine("functional Tests completed.");
			Tests.QuatPerfTest.DoTest();
			Vectorized.QuatPerfTest.DoTest();
			ByRefVector.QuatPerfTest.DoTest();
			Console.Write("Press any key to continue...");
			Console.ReadKey();
			Console.WriteLine();
        }
    }
}