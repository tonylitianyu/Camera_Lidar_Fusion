#include "lidarRead.h"

namespace lidarRead
{	
	bool isLidarOpen(Urg_driver& urg)
	{
		return urg.is_open();
	}
	void closeLidar(Urg_driver& urg)
	{
		urg.close();
	}
	int initLidar(int argc, const char **argv, Urg_driver& urg)
	{
			Connection_information information(argc, argv);
			do
			{
					if (!urg.open(information.device_or_ip_name(), information.baudrate_or_port_number(), information.connection_type()))
					{
							std::cout << "Urg_driver::open(): " << information.device_or_ip_name() << ": " << urg.what() << std::endl;
					} 
					if (urg.max_data_size()<0) urg.close();
			}while(urg.max_data_size()<0);

	#if 1
			urg.set_scanning_parameter(urg.deg2step(-135), urg.deg2step(+135), 0);
	#endif
			enum { Capture_times = 1 };
			//urg.start_measurement(Urg_driver::Distance, Urg_driver::Infinity_times, 0);
			urg.start_measurement(Urg_driver::Distance_intensity, Urg_driver::Infinity_times, 0);
        
        return 1;
	}

	void getLidar(Urg_driver& urg, std::vector<long> &data_distance, std::vector<unsigned short> &data_intensity)
	{
		long time_stamp = 0;
		int success = 0;
		success = urg.get_distance_intensity(data_distance, data_intensity, &time_stamp);
		if(!success)
		{
				std::cout << "Urg_driver::get_distance(): " << urg.what() << std::endl;
				data_distance.push_back(-1);
		}     
		else
		{
				data_distance.push_back(time_stamp);
		}
	}
	void *lidarReading( void *ptr )
	{
		Urg_driver urg0;
		thdata *data;
		data = (thdata *) ptr;
		int b_loop = 1;	
		std::vector<long> distance;
		std::vector<unsigned short> data_intensity;

		data->mtx.lock();
		char const *arr0[] = { "lidar", data->connection_type.c_str(), data->ip_or_portname.c_str() };	
		initLidar(3, arr0, urg0);
		getLidar(urg0, data->distance, data->intensity);
		data->b_loop = 1;
		data->distance.pop_back();
		data->mtx.unlock();

		while(b_loop)
		{
			try
			{
				getLidar(urg0, distance, data_intensity);				
				if (distance.back()==-1)
				{
					std::cout << "ERROR";
					closeLidar(urg0);
					initLidar(3,arr0,urg0);
					getLidar(urg0, distance, data_intensity);	
				}  
			}
			catch (...)
			{
				if (urg0.max_data_size() < 0) 
				{
					std::cout << "\n\n\n!!!ERROR LIDAR!!!\n\n\n";
					b_loop = 0;					
				}      
			}  				
			data->mtx.lock();
			std::copy(distance.begin(), distance.end()-1, data->distance.begin());
			std::copy(data_intensity.begin(), data_intensity.end()-1, data->intensity.begin());
			data->timestamp = distance.back();	
			if(b_loop) b_loop = data->b_loop;
			else data->b_loop = b_loop;
			data->mtx.unlock();
		}
		closeLidar(urg0);
        return NULL;
	}	
}
