from cx_Freeze import setup, Executable 
  
setup(name = "heyulong" , 
      version = "0.1.0" , 
      description = "" , 
      executables = [Executable("runner.py")]) 