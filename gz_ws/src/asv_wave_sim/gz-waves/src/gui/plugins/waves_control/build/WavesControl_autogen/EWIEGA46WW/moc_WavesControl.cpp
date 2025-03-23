/****************************************************************************
** Meta object code from reading C++ file 'WavesControl.hh'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../WavesControl.hh"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'WavesControl.hh' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_gz__sim__GZ_WAVES_VERSION_NAMESPACE__WavesControl_t {
    QByteArrayData data[12];
    char stringdata0[219];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_gz__sim__GZ_WAVES_VERSION_NAMESPACE__WavesControl_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_gz__sim__GZ_WAVES_VERSION_NAMESPACE__WavesControl_t qt_meta_stringdata_gz__sim__GZ_WAVES_VERSION_NAMESPACE__WavesControl = {
    {
QT_MOC_LITERAL(0, 0, 49), // "gz::sim::GZ_WAVES_VERSION_NAM..."
QT_MOC_LITERAL(1, 50, 23), // "OnShowWaterPatchMarkers"
QT_MOC_LITERAL(2, 74, 0), // ""
QT_MOC_LITERAL(3, 75, 8), // "_checked"
QT_MOC_LITERAL(4, 84, 22), // "OnShowWaterlineMarkers"
QT_MOC_LITERAL(5, 107, 30), // "OnShowSubmergedTriangleMarkers"
QT_MOC_LITERAL(6, 138, 15), // "UpdateWindSpeed"
QT_MOC_LITERAL(7, 154, 10), // "_windSpeed"
QT_MOC_LITERAL(8, 165, 15), // "UpdateWindAngle"
QT_MOC_LITERAL(9, 181, 10), // "_windAngle"
QT_MOC_LITERAL(10, 192, 15), // "UpdateSteepness"
QT_MOC_LITERAL(11, 208, 10) // "_steepness"

    },
    "gz::sim::GZ_WAVES_VERSION_NAMESPACE::WavesControl\0"
    "OnShowWaterPatchMarkers\0\0_checked\0"
    "OnShowWaterlineMarkers\0"
    "OnShowSubmergedTriangleMarkers\0"
    "UpdateWindSpeed\0_windSpeed\0UpdateWindAngle\0"
    "_windAngle\0UpdateSteepness\0_steepness"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_gz__sim__GZ_WAVES_VERSION_NAMESPACE__WavesControl[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   44,    2, 0x0a /* Public */,
       4,    1,   47,    2, 0x0a /* Public */,
       5,    1,   50,    2, 0x0a /* Public */,
       6,    1,   53,    2, 0x0a /* Public */,
       8,    1,   56,    2, 0x0a /* Public */,
      10,    1,   59,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Double,    7,
    QMetaType::Void, QMetaType::Double,    9,
    QMetaType::Void, QMetaType::Double,   11,

       0        // eod
};

void gz::sim::GZ_WAVES_VERSION_NAMESPACE::WavesControl::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<WavesControl *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->OnShowWaterPatchMarkers((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->OnShowWaterlineMarkers((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->OnShowSubmergedTriangleMarkers((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->UpdateWindSpeed((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 4: _t->UpdateWindAngle((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 5: _t->UpdateSteepness((*reinterpret_cast< double(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject gz::sim::GZ_WAVES_VERSION_NAMESPACE::WavesControl::staticMetaObject = { {
    QMetaObject::SuperData::link<gz::sim::GuiSystem::staticMetaObject>(),
    qt_meta_stringdata_gz__sim__GZ_WAVES_VERSION_NAMESPACE__WavesControl.data,
    qt_meta_data_gz__sim__GZ_WAVES_VERSION_NAMESPACE__WavesControl,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *gz::sim::GZ_WAVES_VERSION_NAMESPACE::WavesControl::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *gz::sim::GZ_WAVES_VERSION_NAMESPACE::WavesControl::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_gz__sim__GZ_WAVES_VERSION_NAMESPACE__WavesControl.stringdata0))
        return static_cast<void*>(this);
    return gz::sim::GuiSystem::qt_metacast(_clname);
}

int gz::sim::GZ_WAVES_VERSION_NAMESPACE::WavesControl::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = gz::sim::GuiSystem::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
