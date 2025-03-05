PROJECT_PATH=./OpenXRKit/OpenXRKit.xcodeproj
SCHEME=openxr_loader
CONFIG=Release
PRODUCT=libopenxr_loader.a
DERIVED_DATA=./build-xcframework
STAGING_DIR=./lib-xcframework
OUTPUT_PATH=$DERIVED_DATA/Build/Products/$CONFIG/$PRODUCT
FINAL_PRODUCT_PATH=./OpenXRLoader.xcframework

rm -r $FINAL_PRODUCT_PATH

mkdir -p $DERIVED_DATA
mkdir -p $STAGING_DIR/macosx
mkdir -p $STAGING_DIR/iphoneos
mkdir -p $STAGING_DIR/iphonesimulator
mkdir -p $STAGING_DIR/xros
mkdir -p $STAGING_DIR/xrsimulator

xcodebuild build -project $PROJECT_PATH -sdk macosx -scheme $SCHEME -configuration $CONFIG -derivedDataPath $DERIVED_DATA BUILD_LIBRARY_FOR_DISTRIBUTION=YES
mv $DERIVED_DATA/Build/Products/$CONFIG/$PRODUCT $STAGING_DIR/macosx
xcodebuild build -project $PROJECT_PATH -sdk iphoneos -scheme $SCHEME -configuration $CONFIG -derivedDataPath $DERIVED_DATA BUILD_LIBRARY_FOR_DISTRIBUTION=YES
mv $DERIVED_DATA/Build/Products/$CONFIG-iphoneos/$PRODUCT $STAGING_DIR/iphoneos
xcodebuild build -project $PROJECT_PATH -sdk iphonesimulator -scheme $SCHEME -configuration $CONFIG -derivedDataPath $DERIVED_DATA BUILD_LIBRARY_FOR_DISTRIBUTION=YES
mv $DERIVED_DATA/Build/Products/$CONFIG-iphonesimulator/$PRODUCT $STAGING_DIR/iphonesimulator
xcodebuild build -project $PROJECT_PATH -sdk xros -scheme $SCHEME -configuration $CONFIG -derivedDataPath $DERIVED_DATA BUILD_LIBRARY_FOR_DISTRIBUTION=YES
mv $DERIVED_DATA/Build/Products/$CONFIG-xros/$PRODUCT $STAGING_DIR/xros
xcodebuild build -project $PROJECT_PATH -sdk xrsimulator -scheme $SCHEME -configuration $CONFIG -derivedDataPath $DERIVED_DATA BUILD_LIBRARY_FOR_DISTRIBUTION=YES
mv $DERIVED_DATA/Build/Products/$CONFIG-xrsimulator/$PRODUCT $STAGING_DIR/xrsimulator

HEADER_PATH=$DERIVED_DATA/Build/Products/$CONFIG/usr/local/include

xcodebuild -create-xcframework \
 -library $STAGING_DIR/macosx/$PRODUCT -headers $HEADER_PATH \
 -library $STAGING_DIR/iphoneos/$PRODUCT -headers $HEADER_PATH \
 -library $STAGING_DIR/iphonesimulator/$PRODUCT -headers $HEADER_PATH \
 -library $STAGING_DIR/xros/$PRODUCT -headers $HEADER_PATH \
 -library $STAGING_DIR/xrsimulator/$PRODUCT -headers $HEADER_PATH \
 -output $FINAL_PRODUCT_PATH

rm -r $STAGING_DIR
rm -r $DERIVED_DATA
