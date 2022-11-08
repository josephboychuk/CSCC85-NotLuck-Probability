# Zips the files for submission
ZIP_NAME="ParticleFilters_NotLuckProbability.zip"
# We only want all .c, .h, and compile.sh in the submission
zip -r $ZIP_NAME . -i \*.c \*.h compile.sh
