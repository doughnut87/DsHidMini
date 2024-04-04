using System;
using System.Globalization;
using System.Windows.Data;
using Nefarius.DsHidMini.Drivers;

namespace Nefarius.DsHidMini.Util.WPF
{
    public class HidModeComboBoxDisableConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if (value == null)
                return null;

            //
            // Switching to this mode while under the "wrong" INF will produce 
            // very confusing results, so it is disabled for user selection
            // 
            /* return (DsHidDeviceMode) value == DsHidDeviceMode.XInputHIDCompatible; */
            return false;
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }

    public class ByteToPercentConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            var byteValue = value is byte b ? b : (byte)0;

            return (int)((double)byteValue * 100/255.0);
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }
}