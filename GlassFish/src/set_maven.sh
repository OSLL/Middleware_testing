mvn install:install-file -Dfile=$IMQ_HOME/lib/jms.jar -DgroupId=com.sun.glassfish -DartifactId=jms -Dversion=1.0 -Dpackaging=jar
mvn install:install-file -Dfile=$IMQ_HOME/lib/imq.jar -DgroupId=com.sun.glassfish -DartifactId=imq -Dversion=1.0 -Dpackaging=jar
