from gunnar import testing
sk = testing.Sketch("stop")
sk.code = "void setup() { ; }; void loop() { ; }"
sk.makeFiles()
sk.upload()
