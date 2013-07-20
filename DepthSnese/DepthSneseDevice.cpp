/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
#include "Driver/OniDriverAPI.h"
#include "XnLib.h"
#include "XnHash.h"
#include "XnEvent.h"
#include "XnPlatform.h"

#include <DepthSense.hxx>
#include <algorithm>
#include <map>

#define DEPTHSENSE_COLOR_RESOLUTION_X 640
#define DEPTHSENSE_COLOR_RESOLUTION_Y 480
#define DEPTHSENSE_DEPTH_RESOLUTION_X 320
#define DEPTHSENSE_DEPTH_RESOLUTION_Y 240

class OzStream : public oni::driver::StreamBase
{
public:
	~OzStream()
	{
		stop();
	}

	xnl::OSEvent m_osEvent;

	OniStatus start()
	{
		m_osEvent.Create(TRUE);

		xnOSCreateThread(threadFunc, this, &m_threadHandle);

		return ONI_STATUS_OK;
	}

	void stop()
	{
		m_running = false;
	}

	virtual OniStatus SetVideoMode(OniVideoMode*) = 0;
	virtual OniStatus GetVideoMode(OniVideoMode* pVideoMode) = 0;

	OniStatus getProperty(int propertyId, void* data, int* pDataSize)
	{
		if (propertyId == ONI_STREAM_PROPERTY_VIDEO_MODE)
		{
			if (*pDataSize != sizeof(OniVideoMode))
			{
				printf("Unexpected size: %d != %d\n", *pDataSize, (int)sizeof(OniVideoMode));
				return ONI_STATUS_ERROR;
			}
			return GetVideoMode((OniVideoMode*)data);
		}

		return ONI_STATUS_NOT_IMPLEMENTED;
	}

	OniStatus setProperty(int propertyId, const void* data, int dataSize)
	{
		if (propertyId == ONI_STREAM_PROPERTY_VIDEO_MODE)
		{
			if (dataSize != sizeof(OniVideoMode))
			{
				printf("Unexpected size: %d != %d\n", dataSize, (int)sizeof(OniVideoMode));
				return ONI_STATUS_ERROR;
			}
			return SetVideoMode((OniVideoMode*)data);
		}

		return ONI_STATUS_NOT_IMPLEMENTED;
	}

	virtual void Mainloop() = 0;
protected:
	// Thread
	static XN_THREAD_PROC threadFunc(XN_THREAD_PARAM pThreadParam)
	{
		OzStream* pStream = (OzStream*)pThreadParam;
		pStream->m_running = true;
		pStream->Mainloop();

		XN_THREAD_PROC_RETURN(XN_STATUS_OK);
	}


	int singleRes(int x, int y) {
		OniVideoMode mode;
		GetVideoMode( &mode );

		return y * mode.resolutionX + x;
	}

	bool m_running;

	XN_THREAD_HANDLE m_threadHandle;

};

class OzDepthStream : public OzStream
{
public:

	OzDepthStream( DepthSense::Context& context, DepthSense::Node node )
		: m_context( context )
	{
		m_depthNode = node.as<DepthSense::DepthNode>();
		configureDepthNode();
		m_context.registerNode( node );

		m_nodeMap[m_depthNode] = this;
	}

	void configureDepthNode()
	{
		m_depthNode.newSampleReceivedEvent().connect(&onNewDepthSample);

		DepthSense::DepthNode::Configuration config = m_depthNode.getConfiguration();
		config.frameFormat = DepthSense::FRAME_FORMAT_QVGA;
		config.framerate = 30;
		config.mode = DepthSense::DepthNode::CAMERA_MODE_CLOSE_MODE;
		config.saturation = true;

		m_depthNode.setEnableVertices(true);
		m_depthNode.setEnableDepthMap( true );
		m_depthNode.setEnableAccelerometer( true );

		try  {
			m_context.requestControl(m_depthNode,0);

			m_depthNode.setConfiguration(config);
		}
		catch (std::exception& e) {
			fprintf( stderr, "exception : %s\n", e.what() );
		}

		int32_t w, h;
		DepthSense::FrameFormat_toResolution( config.frameFormat, &w, &h );
		m_data.resize( w * h );
	}

	static void onNewDepthSample(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data)
	{
		OzDepthStream* pStream = m_nodeMap[node];
		if ( pStream ) {
			//fprintf( stderr, "onNewDepthSample\n" );

			xnOSMemCopy( &pStream->m_data[0], data.depthMap,  pStream->m_data.size() * 2 );

			pStream->m_osEvent.Set();
		}
		else {
			fprintf( stderr, "onNewDepthSample : no node poiter" );
		}
	}

	OniStatus SetVideoMode(OniVideoMode*) {return ONI_STATUS_NOT_IMPLEMENTED;}
	OniStatus GetVideoMode(OniVideoMode* pVideoMode)
	{
		pVideoMode->pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
		pVideoMode->fps = 30;
		pVideoMode->resolutionX = DEPTHSENSE_DEPTH_RESOLUTION_X;
		pVideoMode->resolutionY = DEPTHSENSE_DEPTH_RESOLUTION_Y;
		return ONI_STATUS_OK;
	}

private:

	void Mainloop()
	{
		int frameId = 1;
		while (m_running)
		{
			m_osEvent.Wait(XN_WAIT_INFINITE);
			m_osEvent.Reset();

			OniFrame* pFrame = getServices().acquireFrame();
			if (pFrame == NULL) {printf("Didn't get frame...\n"); continue;}

			// Fill frame
			xnOSMemSet(pFrame->data, 0, pFrame->dataSize);

			pFrame->frameIndex = frameId;

			pFrame->videoMode.pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
			pFrame->videoMode.resolutionX = DEPTHSENSE_DEPTH_RESOLUTION_X;
			pFrame->videoMode.resolutionY = DEPTHSENSE_DEPTH_RESOLUTION_Y;
			pFrame->videoMode.fps = 30;

			pFrame->width = DEPTHSENSE_DEPTH_RESOLUTION_X;
			pFrame->height = DEPTHSENSE_DEPTH_RESOLUTION_Y;

			xnOSMemCopy( pFrame->data, &m_data[0], m_data.size() * 2 );

			pFrame->cropOriginX = pFrame->cropOriginY = 0;
			pFrame->croppingEnabled = FALSE;

			pFrame->sensorType = ONI_SENSOR_DEPTH;
			pFrame->stride = DEPTHSENSE_DEPTH_RESOLUTION_X*sizeof(OniDepthPixel);
			pFrame->timestamp = frameId * 33000;

			raiseNewFrame(pFrame);
			getServices().releaseFrame(pFrame);

			frameId++;
		}
	}

	DepthSense::Context& m_context;
	DepthSense::DepthNode m_depthNode;

	std::vector<int16_t> m_data;
	static std::map<DepthSense::DepthNode, OzDepthStream*> m_nodeMap;
};

/*static*/ std::map<DepthSense::DepthNode, OzDepthStream*> OzDepthStream::m_nodeMap;


class OzImageStream : public OzStream
{
public:

	OzImageStream( DepthSense::Context& context, DepthSense::Node node )
		: m_context( context )
	{
		m_colorNode = node.as<DepthSense::ColorNode>();
        configureColorNode();
        m_context.registerNode(node);

		m_nodeMap[m_colorNode] = this;
	}

	// DepthSense SDK �̃J���[�X�g���[�����쐬
	void configureColorNode()
	{
		// connect new color sample handler
		m_colorNode.newSampleReceivedEvent().connect(&onNewColorSample);

		DepthSense::ColorNode::Configuration config = m_colorNode.getConfiguration();
		config.frameFormat = DepthSense::FRAME_FORMAT_VGA;
		config.compression = DepthSense::COMPRESSION_TYPE_MJPEG;
		config.powerLineFrequency = DepthSense::POWER_LINE_FREQUENCY_50HZ;
		config.framerate = 30;

		m_colorNode.setEnableColorMap(true);

		try {
			m_context.requestControl( m_colorNode, 0 );

			m_colorNode.setConfiguration(config);

			fprintf( stderr, "Image node config is success\n" );
		}
		catch (std::exception& e) {
			fprintf( stderr, "exception : %s\n", e.what() );
		}

		int32_t w, h;
		DepthSense::FrameFormat_toResolution( config.frameFormat, &w, &h );
		m_data.resize( w * h * 3 );
	}

	// �J���[�X�g���[���̃t���[���X�V�R�[���o�b�N�֐�
	static void onNewColorSample( DepthSense::ColorNode node,
		                          DepthSense::ColorNode::NewSampleReceivedData data)
	{
		OzImageStream* pStream = m_nodeMap[node];
		if ( pStream ) {
			//fprintf( stderr, "onNewColorSample\n" );

			xnOSMemCopy( &pStream->m_data[0], data.colorMap,  pStream->m_data.size() );

			pStream->m_osEvent.Set();
		}
		else {
			fprintf( stderr, "onNewColorSample : no node poiter\n" );
		}
	}

	OniStatus SetVideoMode(OniVideoMode*) {return ONI_STATUS_NOT_IMPLEMENTED;}
	OniStatus GetVideoMode(OniVideoMode* pVideoMode)
	{
		pVideoMode->pixelFormat = ONI_PIXEL_FORMAT_RGB888;
		pVideoMode->fps = 30;
		pVideoMode->resolutionX = DEPTHSENSE_COLOR_RESOLUTION_X;
		pVideoMode->resolutionY = DEPTHSENSE_COLOR_RESOLUTION_Y;
		return ONI_STATUS_OK;
	}

private:

	void Mainloop()
	{
		int frameId = 1;
		while (m_running)
		{
			m_osEvent.Wait(XN_WAIT_INFINITE);
			m_osEvent.Reset();

			OniFrame* pFrame = getServices().acquireFrame();
			if (pFrame == NULL) {printf("Didn't get frame...\n"); continue;}

			// Fill frame
			xnOSMemSet(pFrame->data, 0, pFrame->dataSize);

			pFrame->frameIndex = frameId;

			pFrame->videoMode.pixelFormat = ONI_PIXEL_FORMAT_RGB888;
			pFrame->videoMode.resolutionX = DEPTHSENSE_COLOR_RESOLUTION_X;
			pFrame->videoMode.resolutionY = DEPTHSENSE_COLOR_RESOLUTION_Y;
			pFrame->videoMode.fps = 30;

			pFrame->width = DEPTHSENSE_COLOR_RESOLUTION_X;
			pFrame->height = DEPTHSENSE_COLOR_RESOLUTION_Y;

			xnOSMemCopy( pFrame->data, &m_data[0], m_data.size() );

			pFrame->cropOriginX = pFrame->cropOriginY = 0;
			pFrame->croppingEnabled = FALSE;

			pFrame->sensorType = ONI_SENSOR_COLOR;
			pFrame->stride = DEPTHSENSE_COLOR_RESOLUTION_X*sizeof(OniDepthPixel);
			pFrame->timestamp = frameId * 33000;

			raiseNewFrame(pFrame);
			getServices().releaseFrame(pFrame);

			frameId++;
		}
	}

	DepthSense::Context& m_context;
	DepthSense::ColorNode m_colorNode;

	std::vector<unsigned char> m_data;
	static std::map<DepthSense::ColorNode, OzImageStream*> m_nodeMap;
};

/*static*/ std::map<DepthSense::ColorNode, OzImageStream*> OzImageStream::m_nodeMap;

class OzDevice : public oni::driver::DeviceBase
{
public:
	OzDevice( OniDeviceInfo* pInfo, oni::driver::DriverServices& driverServices,
		      DepthSense::Context& context, DepthSense::Device& device )
		: m_pInfo(pInfo)
		, m_driverServices(driverServices)
		, m_context( context )
		, m_device( device )
	{
		m_numSensors = 2;

		m_sensors[0].pSupportedVideoModes = XN_NEW_ARR(OniVideoMode, 1);
		m_sensors[0].sensorType = ONI_SENSOR_DEPTH;
		m_sensors[0].numSupportedVideoModes = 1;
		m_sensors[0].pSupportedVideoModes[0].pixelFormat = ONI_PIXEL_FORMAT_DEPTH_1_MM;
		m_sensors[0].pSupportedVideoModes[0].fps = 30;
		m_sensors[0].pSupportedVideoModes[0].resolutionX = DEPTHSENSE_DEPTH_RESOLUTION_X;
		m_sensors[0].pSupportedVideoModes[0].resolutionY = DEPTHSENSE_DEPTH_RESOLUTION_Y;

		m_sensors[1].pSupportedVideoModes = XN_NEW_ARR(OniVideoMode, 1);
		m_sensors[1].sensorType = ONI_SENSOR_COLOR;
		m_sensors[1].numSupportedVideoModes = 1;
		m_sensors[1].pSupportedVideoModes[0].pixelFormat = ONI_PIXEL_FORMAT_RGB888;
		m_sensors[1].pSupportedVideoModes[0].fps = 30;
		m_sensors[1].pSupportedVideoModes[0].resolutionX = DEPTHSENSE_COLOR_RESOLUTION_X;
		m_sensors[1].pSupportedVideoModes[0].resolutionY = DEPTHSENSE_COLOR_RESOLUTION_Y;

	}

	OniDeviceInfo* GetInfo()
	{
		return m_pInfo;
	}

	OniStatus getSensorInfoList(OniSensorInfo** pSensors, int* numSensors)
	{
		*numSensors = m_numSensors;
		*pSensors = m_sensors;

		return ONI_STATUS_OK;
	}

	oni::driver::StreamBase* createStream(OniSensorType sensorType)
	{
		if (sensorType == ONI_SENSOR_DEPTH)
		{
			// Depth �m�[�h��T��
	        std::vector<DepthSense::Node> nodes = m_device.getNodes();
			auto it = std::find_if( nodes.begin(), nodes.end(), []( DepthSense::Node& val ) {
				return val.is<DepthSense::DepthNode>();
			} );

			// Depth �X�g���[�����쐬����
			if ( it != nodes.end() ) {
				OzDepthStream* pImage = XN_NEW( OzDepthStream, m_context, *it );
				return pImage;
			}
		}
		if (sensorType == ONI_SENSOR_COLOR)
		{
			// �J���[�m�[�h��T��
	        std::vector<DepthSense::Node> nodes = m_device.getNodes();
			auto it = std::find_if( nodes.begin(), nodes.end(), []( DepthSense::Node& val ) {
				return val.is<DepthSense::ColorNode>();
			} );

			// �J���[�X�g���[�����쐬����
			if ( it != nodes.end() ) {
				OzImageStream* pImage = XN_NEW( OzImageStream, m_context, *it );
				return pImage;
			}
		}

		m_driverServices.errorLoggerAppend("OzDevice: Can't create a stream of type %d", sensorType);
		return NULL;
	}

	void destroyStream(oni::driver::StreamBase* pStream)
	{
		XN_DELETE(pStream);
	}

	OniStatus  getProperty(int propertyId, void* data, int* pDataSize)
	{
		OniStatus rc = ONI_STATUS_OK;

		switch (propertyId)
		{
		case ONI_DEVICE_PROPERTY_DRIVER_VERSION:
			{
				if (*pDataSize == sizeof(OniVersion))
				{
					OniVersion* version = (OniVersion*)data;
					version->major = version->minor = version->maintenance = version->build = 2;
				}
				else
				{
					m_driverServices.errorLoggerAppend("Unexpected size: %d != %d\n", *pDataSize, sizeof(OniVersion));
					rc = ONI_STATUS_ERROR;
				}
			}
			break;
		default:
			m_driverServices.errorLoggerAppend("Unknown property: %d\n", propertyId);
			rc = ONI_STATUS_ERROR;
		}
		return rc;
	}
private:
	OzDevice(const OzDevice&);
	void operator=(const OzDevice&);

	OniDeviceInfo* m_pInfo;
	int m_numSensors;
	OniSensorInfo m_sensors[10];
	oni::driver::DriverServices& m_driverServices;

	DepthSense::Context& m_context;
	DepthSense::Device m_device;
};


class OzDriver : public oni::driver::DriverBase
{
public:
	OzDriver(OniDriverServices* pDriverServices) : DriverBase(pDriverServices)
	{}

	// �h���C�o������������
	virtual OniStatus initialize(
		oni::driver::DeviceConnectedCallback connectedCallback,
		oni::driver::DeviceDisconnectedCallback disconnectedCallback,
		oni::driver::DeviceStateChangedCallback deviceStateChangedCallback,
		void* pCookie)
	{
		oni::driver::DriverBase::initialize(connectedCallback, disconnectedCallback, deviceStateChangedCallback, pCookie);

		// DepthSense �f�o�C�X�ɐڑ�����
		m_context = DepthSense::Context::create("localhost");

		// �ڑ�����Ă���f�o�C�X�̏����쐬����
		m_depthSenseDevices = m_context.getDevices();
		for ( auto it = m_depthSenseDevices.begin(); it != m_depthSenseDevices.end(); ++it ) {
			// �f�o�C�X�����R�s�[����
			OniDeviceInfo* pInfo = XN_NEW(OniDeviceInfo);
			xnOSStrCopy(pInfo->vendor, "DepthSense", ONI_MAX_STR);
			xnOSStrCopy(pInfo->name, DepthSense::Device::Model_toString( it->getModel() ).c_str(), ONI_MAX_STR);
			xnOSStrCopy(pInfo->uri, DepthSense::Device::Model_toString( it->getModel() ).c_str(), ONI_MAX_STR);

			// �f�o�C�X����o�^����
			m_devices[pInfo] = NULL;

			// �f�o�C�X�̐ڑ���ʒm����
			deviceConnected(pInfo);
			deviceStateChanged(pInfo, 0);
		}

		// �f�o�C�X�̃��C�����[�v�p�X���b�h�𐶐�����
		xnOSCreateThread(threadFunc, this, &m_threadHandle);

		return ONI_STATUS_OK;
	}

	virtual oni::driver::DeviceBase* deviceOpen(const char* uri, const char* /*mode*/)
	{
		for ( auto iter = m_devices.Begin(); iter != m_devices.End(); ++iter) {
			if (xnOSStrCmp(iter->Key()->uri, uri) == 0) {
				// Found
				if (iter->Value() != NULL) {
					// already using
					return iter->Value();
				}

				// URI�̃f�o�C�X��T��
				auto it = std::find_if( m_depthSenseDevices.begin(), m_depthSenseDevices.end(), 
					[&uri]( DepthSense::Device& device ) {
						return DepthSense::Device::Model_toString( device.getModel() ) == uri;
					} );
				if ( it == m_depthSenseDevices.end() ) {
					throw std::runtime_error( "�f�o�C�X��������܂���" );
				}

				// �f�o�C�X�C���X�^���X�𐶐�����
				OzDevice* pDevice = XN_NEW(OzDevice, iter->Key(), getServices(), m_context, *it );
				iter->Value() = pDevice;
				return pDevice;
			}
		}

		getServices().errorLoggerAppend("Looking for '%s'", uri);
		return NULL;
	}

	virtual void deviceClose(oni::driver::DeviceBase* pDevice)
	{
		for (xnl::Hash<OniDeviceInfo*, oni::driver::DeviceBase*>::Iterator iter = m_devices.Begin(); iter != m_devices.End(); ++iter)
		{
			if (iter->Value() == pDevice)
			{
				iter->Value() = NULL;
				XN_DELETE(pDevice);
				return;
			}
		}

		// not our device?!
		XN_ASSERT(FALSE);
	}

	void shutdown() {}

protected:

	// �f�o�C�X�̃��C�����[�v�p�X���b�h
	static XN_THREAD_PROC threadFunc(XN_THREAD_PARAM pThreadParam)
	{
		//fprintf( stderr, "context is running\n" );

		OzDriver* pDriver = (OzDriver*)pThreadParam;
		pDriver->m_context.startNodes();
		pDriver->m_context.run();
		pDriver->m_context.stopNodes();

		XN_THREAD_PROC_RETURN(XN_STATUS_OK);
	}

	XN_THREAD_HANDLE m_threadHandle;

	xnl::Hash<OniDeviceInfo*, oni::driver::DeviceBase*> m_devices;

	DepthSense::Context m_context;
	std::vector<DepthSense::Device> m_depthSenseDevices;
};

ONI_EXPORT_DRIVER(OzDriver);
